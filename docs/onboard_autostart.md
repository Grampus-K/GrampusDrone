# Orin NX 联网校时后自动启动

更新日期：2026-10-21。本服务现在**只在网络可达、本次开机已完成网络校时、时钟稳定后启动 ROS 定位链路**。离线冷启动会等待，即使 RTC 日期正确也不会直接启动。此要求用于避免 ROS 已经 READY 后，首次 NTP 校时使 IMU 时间戳倒退。

自动启动不会解锁或起飞。服务模板的控制器和规划器默认关闭，正式输出模式由已有配置决定：`LIO_OUTPUT_MODE=vision_pose` 或 `odometry`。这次升级不改变你的 `lio-output.conf`、安装外参和 PX4 参数。完整里程计配置见 [双模式文档](lio_to_mavros_dual_output.md)，现场流程见 [测试方案](localization_field_test_plan.md)。

## 启动条件与断网行为

`start_onboard.sh` 在启动 ROS、Livox、MAVROS、FAST-LIO 之前运行 `wait_for_stable_clock.py`。必须同时通过：

1. TCP 网络探测：默认连接 `connectivity-check.ubuntu.com:80`，包含 DNS 解析，单次最长 5 秒。仅连接成功，不发送 HTTP 请求或上传数据。
2. `systemd-timesyncd.service` 为 active，且存在本次开机的 `/run/systemd/timesync/synchronized` 标记。
3. `timedatectl show --property=NTPSynchronized --value` 返回 `yes`。
4. 系统时间不早于 `MIN_VALID_EPOCH=1735689600`，并在上述条件成立后连续观察稳定至少 10 秒。启动检查中，系统时钟相对单调时钟的步变超过 0.01 秒会重新计时；网络或同步条件丢失也会重新计时。

`/run` 的同步标记会在重启后清除。磁盘上的 `/var/lib/systemd/timesync/clock` 仅用于恢复大致日期，不能替代成功校时。仅连上热点、获得 IP、到达 `network-online.target`、系统日期看似正确，都不等于这四项通过。

这是针对当前 Orin 使用的 **systemd-timesyncd** 实现的门槛。不把 chrony、手动改时间或旧日期缓存当作等价证据；如果更换校时服务，需要适配校时检查，不能伪造同步标记。NTP 服务未启用、DNS/TCP 探测失败、UDP 123 被网络阻断等情况都将继续等待。

**只限制启动，不以网络作为运行期停机条件：**

| 场景 | 行为 |
|---|---|
| 开机没有网络 | 服务等待，不启动 ROS 定位节点 |
| 已连 Wi-Fi，但探测不通或 NTP 未同步 | 继续等待 |
| 联网且本次开机校时完成 | 稳定窗口通过后自动启动，不需重新启动服务 |
| 稳定窗口内断网或时间跳变 | 重新等待完整稳定窗口 |
| 定位已运行后断网 | 此门槛已退出，不因此停止或重启定位 |
| 断网时其他故障触发服务重启 | 新一轮启动会等待网络和校时条件，不能假定可离线恢复 |

网络探测只证明所选主机端口可达，**不证明前端平台登录、WebSocket、业务接口或所有互联网服务可用**。默认使用 Ubuntu 探测主机；平台地址确定后可替换为平台的实际 TCP 主机和端口。热点有认证门户或防火墙时，需要确认实际 NTP 可用。

这不是永久无跳时保证。运行后的网络重连、手动校时或其他校时服务仍需单独验证。不能以“启动条件通过”代替健康检查、融合验证和起飞前检查。

## Ubuntu 更新现有服务

在地面、飞机未解锁、记录已正常停止后执行。不要在运行中的飞行测试内更新或重启：

~~~bash
cd ~/GrampusDrone
git status --short --branch
git pull --ff-only origin master
python3 -m unittest discover -s shfiles/tests -v

sudo systemctl stop grampusdrone.service
sudo cp /etc/systemd/system/grampusdrone.service \
  "/etc/systemd/system/grampusdrone.service.backup-$(date +%Y%m%d_%H%M%S)"
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh
sudo cp shfiles/grampusdrone.service /etc/systemd/system/grampusdrone.service
sudo systemctl daemon-reload
sudo systemctl enable grampusdrone.service
sudo systemctl start grampusdrone.service
sudo journalctl -u grampusdrone.service -b -f
~~~

本次修改仅脚本、服务配置及文档，**已编译工程无需重新 catkin_make**。拉取被本地修改阻止时先保留并核对，不使用 `git reset --hard`。第一次安装时省略“备份旧服务”那一条；如果是新工程，先按 [双模式文档](lio_to_mavros_dual_output.md) 完成依赖和编译。

已有 `/etc/systemd/system/grampusdrone.service.d/*.conf` 会继续覆盖模板中的对应字段。特别检查是否有人覆盖了 `ExecStart`，否则可能绕过启动入口：

~~~bash
sudo systemctl cat grampusdrone.service --no-pager
sudo systemctl show grampusdrone.service --no-pager \
  -p ExecStart -p Environment -p NRestarts -p MainPID
~~~

保留已经验证的 `LIO_OUTPUT_MODE=odometry`、`LIO_ODOMETRY_PREVIEW=false`、`START_CONTROLLER=false`、`START_PLANNER=false`。模板不负责改你的模式 override。

## 查看等待原因

~~~bash
systemctl status grampusdrone.service --no-pager -l
sudo journalctl -u grampusdrone.service -b -n 80 --no-pager
systemctl is-active systemd-timesyncd.service
timedatectl show --property=NTPSynchronized --value
ls -l /run/systemd/timesync/synchronized
sudo journalctl -u systemd-timesyncd.service -b --no-pager
~~~

日志示例：

~~~text
Waiting for network connectivity-check.ubuntu.com:80, this boot's NTP synchronization, then 10.0 s of stable clock; ROS is not started.
Still waiting: network TCP probe failed: connectivity-check.ubuntu.com:80
Still waiting: waiting for this boot's successful systemd-timesyncd synchronization
Still waiting: clock stability 0.0/10.0 s
Startup gate passed: network reachable; NTP synchronized this boot; clock stable. System clock accepted: ...
~~~

`Type=simple` 下等待进程仍显示 `active (running)`，这不等于 ROS 已启动。离线等待时没有 ROS Master、健康话题和 READY 是预期结果；不要反复重启服务。

如本机使用的确实是 systemd-timesyncd，但被禁用，可在地面、定位服务停止后恢复：

~~~bash
sudo systemctl stop grampusdrone.service
sudo timedatectl set-ntp true
sudo systemctl enable --now systemd-timesyncd.service
systemctl is-active systemd-timesyncd.service
timedatectl status
sudo systemctl start grampusdrone.service
~~~

若实际运行的是 chrony，不要同时开启两套时间服务。这里不自动更换 NTP 后端，也不自动修改 NTP 服务器；当前设备日志已确认使用 systemd-timesyncd。

## 可配置项

用独立 drop-in 修改，避免覆盖已有输出模式配置。设置值时不使用 `http://` 等 URL，只填主机名或 IP：

~~~bash
sudo systemctl edit grampusdrone.service
~~~

例如改为自己的平台地址时，在编辑器填写真实值：

~~~ini
[Service]
Environment=NETWORK_CHECK_HOST=your-platform.example.com
Environment=NETWORK_CHECK_PORT=443
~~~

上面是示例域名，不能原样使用。修改后在地面执行 `sudo systemctl daemon-reload`、`sudo systemctl restart grampusdrone.service`。

| 变量 | 默认值 | 作用 |
|---|---|---|
| NETWORK_CHECK_HOST | connectivity-check.ubuntu.com | TCP 探测主机；不是平台业务健康检查 |
| NETWORK_CHECK_PORT | 80 | 探测端口 |
| NETWORK_CHECK_TIMEOUT | 5 | 单次网络探测总超时，含 DNS，单位秒 |
| CLOCK_STABLE_SECONDS | 10 | 所有前提满足后的稳定观察时长 |
| CLOCK_START_STEP_TOLERANCE | 0.01 | 启动阶段允许的单次观测时钟步变，单位秒 |
| CLOCK_WAIT_TIMEOUT | 0 | 0 表示持续等待；正值超时退出后 systemd 按失败策略重试 |
| MIN_VALID_EPOCH | 1735689600 | 日期下限，不能据此单独证明日期准确 |
| CLOCK_STEP_TOLERANCE | 0.5 | 现有运行期时间监控阈值，独立于启动阈值，本次未改变 |

命令失败和不支持的同步状态不会放行。默认无限等待不会因为没联网而反复拉起整栈；网络探测间隔约 2 秒加命令耗时，单次 systemd 查询最长 3 秒。自定义总等待超时在本轮探测结束后检查，不是精确计时截止。

## 地面验收：替换原离线自动启动要求

保持控制器、规划器关闭，地面未解锁完成：

1. **无网络冷启动**：所有可上网接口不可达，启动后至少观察 60 秒。日志应保持等待，不出现 `Startup gate passed`，无本服务启动的 Livox/MAVROS/FAST-LIO/桥接器，`NRestarts` 不因默认无限等待而增加。不要另外手动启动 ROS。
2. **连接热点**：不手动重启 grampusdrone；观察本次 NTP 同步标记及 `NTPSynchronized=yes`，然后出现稳定窗口和放行日志。首次同步应发生在定位启动之前。
3. **正式模式基线**：加载 ROS 环境后执行下面命令。约 10 Hz、健康 True、READY，旧 pose 无本桥接器发布者。
4. **联网后短暂断网**：仅地面，在 Orin 本地终端观察 120 秒，定位应继续；记录 PID、重启计数、健康和频率。前端断开与定位故障分别记录，不在飞行中断网试验。
5. **重新联网**：仅地面验证，保留校时、时间戳和健康日志。此次启动修复不意味着后续联网绝不会引发校时。
6. **重复三轮断电启动**：按“离线等待 → 联网校时 → 自动就绪 → 120 秒基线”记录；没有网络时自动 READY 不再是验收目标。

~~~bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/GrampusDrone/devel/setup.bash
REQUIRE_SYSTEM_TIME_MONITOR=true bash shfiles/wait_for_stack.sh 120
rosparam get /lio_to_mavros/output_mode
rostopic echo -n 1 /system_time/healthy
rostopic echo -n 1 /lio_to_mavros/healthy
rostopic echo -n 1 /lio_to_mavros/status
rostopic info /mavros/vision_pose/pose
timeout 30s rostopic hz /mavros/odometry/out
sudo systemctl show grampusdrone.service --no-pager -p NRestarts -p MainPID
~~~

上面的正式输出检查适用于 odometry 模式；vision_pose 模式检查 `/Odometry` 和 `/mavros/vision_pose/pose`。启动等待阶段不要先运行需要 ROS 的记录脚本；用 journal 记录启动过程，ROS 就绪后再开始 rosbag。整栈重启后必须重新开始记录，不假定旧 recorder 能继续跨 Master 重启采集。

Windows 本地可运行不依赖 ROS/systemd/网络的逻辑测试：`py -3 -m unittest discover -s shfiles/tests -v`。Ubuntu 同一测试命令用 `python3`。这些测试不替代上述真实 systemd、网络和冷启动验收。

## 运行期保护与手动启动

`monitor_system_time.py` 继续监测超过既有阈值的系统时钟跳变。它在 MAVROS 状态新鲜、PX4 已连接且未解锁时请求整栈重启；已解锁或状态未知时，不通过该分支主动重启。其他进程退出或启动就绪超时仍可能触发 systemd 重启，不能把这理解为所有重启路径都有解锁保护。

完整 odometry 门控在异常后锁止，不因输入恢复而自动重建参考。FAST-LIO 的 respawn 是另一回事，不等于融合参考安全恢复。旧 vision_pose 的保护和预放行恢复策略与完整模式不同，详见 [双模式文档](lio_to_mavros_dual_output.md)。

`sudo systemctl stop grampusdrone.service` 之后才能做独立手动测试，避免两套链路同时运行。推荐仍使用服务入口；直接 `roslaunch` 会绕过联网门槛且不自动启动运行期时间监控。仅手动调用门槛时，可以执行：

~~~bash
python3 shfiles/wait_for_stable_clock.py
~~~

不要因门槛未通过而改用 `ready_go.sh` 或裸 `roslaunch` 绕过。若只是想停止开机自启动，在地面执行：

~~~bash
sudo systemctl disable --now grampusdrone.service
~~~

## 常见问题

- **一直等待网络探测**：检查热点是否有互联网、DNS 和目标 TCP 端口；目标被防火墙阻断时改成已验证可达的主机。TCP 探测成功不代表 NTP 的 UDP 123 可用。
- **有网络但一直等待校时**：查看 timesyncd 日志和同步标记。不能靠手动创建标记、改日期下限或加大容差来放行。
- **权限错误 203/EXEC**：执行 `chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh`，核对服务路径和 User；Python 门槛由 python3 调用，不依赖可执行位。
- **ROS_DISTRO 未绑定变量**：`start_onboard.sh` 使用 `set -Eeo pipefail`，不要在 source ROS 环境前增加 `-u`。
- **服务状态与配置不一致**：`systemctl cat` 看所有 drop-in；修改 unit 后执行 daemon-reload，再在地面重启。仅 git pull 不会更新 /etc 中的服务模板。
- **无网络时其他故障导致退出**：下一次服务启动会等待网络。这是用户选择的联网启动策略的实际边界；它不提供离线故障恢复能力。
