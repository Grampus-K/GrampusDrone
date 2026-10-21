# 完整里程计定位系统：地面与首次飞行测试方案

编写日期：2026-10-20；启动策略更新：2026-10-21。计划执行日期：2026-10-21；延期时以实际日期记录。

**启动策略已调整：必须联网并完成本次开机的 NTP 校时，再通过稳定窗口后启动 ROS。** 原“离线冷启动自动 READY”不再适用，001-B2 改为验证“离线等待 → 联网自动启动”。运行期间断网不主动停定位；详见 [联网自启动说明](onboard_autostart.md)。

适用工程：GrampusDrone，ROS Noetic、MID360、FAST-LIO、单节点 lio_to_mavros、MAVROS 1.20.1。编写时核对代码基线为 `109944f`，PX4 实际版本须现场用 `ver all` 保存。本文中的 PX4 行为解释参考 v1.17.0，实际固件不同则先核对，不照搬参数。

本方案验证的是 **PX4 Position 位置模式下的新定位输入**，不启动 EGO、px4ctrl，不测试 Offboard 或自主规划。正式输入使用 `/mavros/odometry/out`。测试人员按顺序执行，有未满足的前提就停在该阶段；不为赶进度跳过地面检查。

## 1. 当天执行范围与阶段门槛

| 编号 | 内容 | 首日安排 | 进入条件 |
|---|---|---|---|
| 000 | 参数、安装原点、遥控接管、日志与地面运动 | 必做 | 全部准备完成后才允许飞行 |
| 001-A/B | 联网启动、断网运行、三次冷启动联网门槛 | 地面完成 | 未解锁，不改变飞行中时钟 |
| 001-C | 纯外部视觉位置模式短悬停与小幅运动 | 条件通过后执行 | 000、001-A/B 通过，人工接管方式已验证 |
| 002-A/B | GNSS 单独可用性、多源融合与地面室内外搬运 | 条件通过后地面执行 | 有实测可用的独立航向策略，参数配置已确认 |
| 002-C/D | 室外多源短悬停、室内外缓慢过渡 | 可选，不是首日必达目标 | 地面双向过渡、对应单环境悬停均通过 |
| 003 | 外部里程计停止发送、GNSS 继续融合 | **仅地面、拆桨** | GNSS 已真正融合且航向不依赖将中断的 EV |
| 004 | FAST-LIO 重启后的锁止与落地恢复 | **仅地面、拆桨** | 已留存故障前日志，按本文专用流程执行 |
| 延期 | 高空退化、空中故障注入、空中重启、发散后无缝恢复 | **本次不执行** | 另行设计和验证参考衔接、故障检测及恢复机制 |

首日完成 001 即可形成有效测试结果。002 条件不满足就标记“未执行”，不要通过升高高度、进入无特征区域、遮挡传感器等方式强迫 FAST-LIO 退化。低空也不等于故障注入安全，003/004 不安排空中或临时绑绳系留试验。

### 1.1 已有证据与仍未证实的内容

2026-10-15 用户提供的结果已证明：systemd 正式模式可启动；旧 pose 与正式 odometry 输出互斥；约 10 Hz；100 帧同时间戳的几何、杆臂和协方差一致性检查通过；PX4 两个 EKF 实例静止时 EV 位置、高度、航向、速度辅助源有新鲜的融合记录。

这些记录不能替代本次检查，也尚未证明：旋翼振动下稳定性、绝对航向正确、GNSS 与 EV 切换可靠、空中故障恢复可用。`healthy=True`、有话题、有 Position 模式都不是独立的飞行许可。

### 1.2 当前实现的明确边界

- 完整 odometry 保持扫描测量时间戳，约 10 Hz；旧 pose 的 30 Hz 不是新模式的频率目标。
- 完整桥接器输入超时默认 0.5 s，MAVROS state 接收超时默认 3 s；数值是门控配置，不是飞行性能保证。
- 桥接器运行后出现相应故障会锁止；重新收到输入不自动恢复，桥接进程不 respawn。
- **当前 onboard_stack 为 FAST-LIO 设置 `fastlio_respawn=true`，退出后约 2 秒重新拉起。** FAST-LIO 重启不等于桥接器或 PX4 恢复；新地图/原点不可直接接入旧融合链路。
- systemd 使用 `Restart=on-failure`。时间监控的“解锁时禁止自动重启”只针对其自身的时钟故障分支，不能保证所有进程退出路径都不会触发整栈重启。
- ROS nav_msgs/Odometry 没有本链路可用的重置序号传递机制。MAVROS 发出的 `reset_counter=0` **不能证明 FAST-LIO 或 PX4 从未重置**。
- 没有覆盖所有“频率正常但地图错误”的发散检测；不能把停流测试当作地图发散测试通过。

## 2. 场地、分工和停止条件

### 2.1 起飞前现场确认

- [ ] 地面排查、搬运和故障测试拆桨、未解锁；装桨后不再手持测试。
- [ ] 试飞区域合法且隔离，无旁观者、车辆、易损物；室内空间足够，远离玻璃、单调走廊、狭窄门洞、低天花板。若无法留出纠偏和降落余量，取消室内飞行。
- [ ] 飞手只管操纵，观察员看飞行状态和遥测；不让飞手悬停时操作终端。
- [ ] 明确起降区、边界、接管模式、RC 开关及低电量降落安排；使用熟悉的机型和控制设置。
- [ ] RC 控制独立于手机热点。若热点承载 RC、QGC 必需链路、RTK 修正或远程终端，先明确影响；这类断网不是单纯 RTC 测试。
- [ ] QGC 无未解决的解锁/估计器警告，遥控、姿态、气压计、供电和存储正常；不关闭检查来强行解锁。
- [ ] 遥控接管方式已经验证。Position/Loiter/RTL 仍可能依赖定位，不能当作定位失效时的独立后备；Altitude 也依赖有效高度估计。不能熟练使用已配置后备模式则不进行新定位试飞。

不规定通用“最低安全高度”。由飞手根据机体、地面效应、障碍和纠偏空间事先设定短悬停高度与边界；贴地到无法纠偏、临时绑绳、在狭窄门口穿行都不是降低风险的可靠办法。

### 2.2 本次保守验收阈值

以下是本次试验门槛，非传感器精度指标；只能在起飞前根据场地进一步收紧，不在异常后放宽。

| 项目 | 门槛 / 操作 |
|---|---|
| 地面初始稳定 | 静置至少 120 s；status 保持 READY、两个健康话题 True，期间无整栈/FAST-LIO 非预期重启 |
| 发布频率 | 30 s 观察平均约 9–11 Hz；不出现 0.5 s 级输入/输出断流；详细间隔以 rosbag 为准 |
| 地面定点漂移 | 静置 120 s 起终点水平变化不超过 0.20 m、垂直不超过 0.15 m；另检查曲线不能中间大漂移又回来 |
| 地面停稳 | 搬运后静置 10 s，再观察 10 s：速度模长通常低于 0.10 m/s，角速度模长通常低于 0.05 rad/s；持续超限需排查 |
| 首次定点悬停 | 先 20–30 s；无持续单向漂移、明显振荡或突变；定点阶段水平偏离目标 0.5 m 或高度偏离 0.3 m 即结束本架次，场地更窄则更早结束 |
| 融合状态 | 预期辅助源新鲜、被实际融合，无持续创新拒绝；一次异常也记录，连续约 1 s 或反复出现则不推进下一项 |
| 硬中止 | 任何意外加速/倾斜、明显位置或 yaw 跳变、失控趋势、定位失效、不可解释的重置/模式切换、桥接 FAULT、时钟故障、非预期进程重启 |

空中出现硬中止条件时：飞手立即按预先练习的接管方式稳定并落地，观察员报状态；不要等终端读完、不要在空中改参数、重启 ROS 或追查日志。无法保证稳定时执行现场应急处置方案。落地未解锁后保存数据并停止后续更高阶段测试。

## 3. 测试 000：建立可复现的基线

### 3.1 同步工程，保持已验证软件版本

仅在地面执行；如果本次拉取只有本文文档，无需重新编译。若有代码变更，重新完成原文档的编译、catkin 测试、地面回归，再开始本方案。当天不要同时升级飞控固件和定位软件。

```bash
cd ~/GrampusDrone
git status --short --branch
git pull --ff-only origin master
git log -3 --oneline
source /opt/ros/noetic/setup.bash
source /home/orin/livox_ws/devel/setup.bash
source ~/GrampusDrone/devel/setup.bash
```

本地修改阻止 pull 时先保存并核对，禁止 `git reset --hard` 丢弃硬件配置。不要同时运行手动 onboard_stack 和 systemd 整栈。

### 3.2 留存参数与安装定义：未确认则不飞

QGC 导出当前完整参数到测试目录，文件注明“正式 odometry 基线”，另保留旧 vision_pose 的已验证参数备份。MAVLink Console 逐条执行并保存文本：

```text
ver all
param show EKF2_EV_CTRL
param show EKF2_GPS_CTRL
param show EKF2_EV_NOISE_MD
param show EKF2_EVP_NOISE
param show EKF2_EVV_NOISE
param show EKF2_EVA_NOISE
param show EKF2_EV_DELAY
param show EKF2_EV_QMIN
param show EKF2_EV_POS_*
param show EKF2_IMU_POS_*
param show EKF2_HGT_REF
param show EKF2_MAG_TYPE
```

参数不存在时记录固件版本和实际名称，在 QGC 参数说明中核对；不创建同名替代项、不猜数值。完整导出还需留存 RC/数据链/位置丢失和低电量 failsafe 设置，逐项理解现场行为。本文不提供可直接覆盖全部融合参数的掩码值。

安装原点核对表：

| 项目 | 当前定义 |
|---|---|
| B/base_link | 用户定义的无人机中心；必须进一步确认它是否就是 PX4 参数使用的重心参考 |
| LiDAR 点云原点相对 B | `[0, 0, 0.07] m`，FLU、无额外旋转 |
| MID360 内部 IMU 相对 LiDAR | `[0.011, 0.02329, -0.04412] m` |
| FAST-LIO `extrinsic_T` | `[-0.011, -0.02329, 0.04412] m`，`extrinsic_R=I`，`extrinsic_est_en=false` |
| 桥接补偿 I → B | `[-0.011, -0.02329, -0.02588] m`，包括杆臂速度和协方差 |
| 飞控 IMU 相对 B | `[0.04,0,0] m`；由一致的 PX4 原点配置处理，不再叠加进桥接位置 |

只有 B 等于 PX4 重心参考且有效飞控 IMU 位置已确认时，几何才对应 `EV_POS=[0,0,0]`、`IMU_POS=[0.04,0,0]`。否则两者都要换算到同一个参考，不能随手写 `EV_POS_X=-0.04`。这个确认目前不能由已有静止日志替代。详细定义见 [双模式文档](lio_to_mavros_dual_output.md)。

### 3.3 systemd 正式模式基线

当前已切到正式模式时只检查，不重复停启：

```bash
sudo systemctl status grampusdrone.service --no-pager -l
sudo systemctl cat grampusdrone.service --no-pager
sudo systemctl show grampusdrone.service --no-pager \
  -p Environment -p EnvironmentFiles -p NRestarts -p ActiveEnterTimestamp
rosparam get /lio_to_mavros/output_mode
rosparam get /lio_to_mavros/odometry_preview
rosparam get /mapping
rosparam get /lio_to_mavros/full
rosnode list
timeout 5s rostopic echo -n 1 /mavros/state
timeout 5s rostopic echo -n 1 /system_time/healthy
timeout 5s rostopic echo -n 1 /lio_to_mavros/healthy
timeout 5s rostopic echo -n 1 /lio_to_mavros/status
rostopic info /mavros/vision_pose/pose
rostopic info /mavros/odometry/out
```

预期 `odometry / false`、`connected=True / armed=False`、两项健康 True、READY；旧 pose 无发布者，正式 odometry 只有预期桥接器发布者，MAVROS 订阅。环境中 controller/planner 均 false，节点列表也不得出现本次意外启动的控制/规划进程。

`active (running)` 只证明脚本活着，`NRestarts` 只统计 systemd 重启，不统计 FAST-LIO respawn。记录两者及节点 PID/ROS 日志中的进程重启。停止脚本返回 130/42 可使 systemd 显示 failed，需结合是否人为停止和当时日志判断。

```bash
cd ~/GrampusDrone
REQUIRE_SYSTEM_TIME_MONITOR=true bash shfiles/wait_for_stack.sh 120
timeout 30s rostopic hz /fast_lio/odometry_full
timeout 30s rostopic hz /mavros/odometry/out
python3 shfiles/check_lio_bridge_odometry.py \
  --topic /mavros/odometry/out --samples 100 --timeout 60
```

`timeout` 到时结束持续命令是预期行为。就绪脚本会请求已有 PX4 IMU/姿态消息流，不是全功能健康证明。检查脚本 PASS 证明同时间戳数学一致性，不证明安装准确、绝对航向或融合正确。

### 3.4 建立本次记录目录及 ROS 记录

推荐使用仓库里的自动化脚本，避免漏掉参数、服务状态或收尾文件。它会创建与原手册相同的记录目录，采集提交版本、RTC、systemd、ROS 参数和磁盘状态，然后在前台启动 rosbag。测试结束时在同一个终端按 `Ctrl-C`，脚本会停止 rosbag、保存 journal、服务重启计数和 `rosbag info`：

```bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source /home/orin/livox_ws/devel/setup.bash
source ~/GrampusDrone/devel/setup.bash
python3 shfiles/localization_test.py record --test-id 001C_ev_hover
```

断热点测试必须在 Orin 本地终端运行；不要把记录进程依赖在会断开的 SSH 会话上。脚本默认会用 `sudo -v` 请求一次权限，以便读取 RTC、systemd 和 journal；若现场不允许 sudo，可加 `--no-sudo`，但对应文件会缺失，自动检查会标记为 `SKIP`。也可以用 `--duration 120` 做固定时长地面记录。

记录结束后，对脚本打印出的目录执行：

```bash
python3 shfiles/localization_test.py analyze \
  "$HOME/flight_test_logs/实际时间_001C_ev_hover"
```

分析会检查：输入和正式输出是否存在、频率是否在 9–11 Hz、最大间隔是否不超过 0.5 s、时间戳是否单调、输入输出时间戳匹配率、数值有限性、桥接器和系统时钟健康、MAVROS 连接状态，以及记录期间 systemd 是否重启。结果同时保存为 `analysis.json`，终端只打印 PASS/FAIL 和关键范围。自动 PASS 只说明记录链路与数据完整性通过；悬停是否稳定、是否发生位置/yaw 跳变、PX4 是否实际融合以及飞手是否安全接管，仍必须结合现场记录、QGC 和 ULog 判定。

如果脚本不可用，下面的手动流程仍可作为后备；`RUN_DIR` 是该终端变量，换终端须重新指定实际路径：

```bash
TEST_ID=001A_online_ground  # 每个子项改名，例如 001C_ev_hover
RUN_DIR="$HOME/flight_test_logs/$(date +%Y%m%d_%H%M%S)_${TEST_ID}"
mkdir -p "$RUN_DIR"
cd ~/GrampusDrone
git rev-parse HEAD > "$RUN_DIR/commit.txt"
git status --short > "$RUN_DIR/git-status.txt"
date -Is > "$RUN_DIR/start-time.txt"
cat /proc/sys/kernel/random/boot_id > "$RUN_DIR/boot-id.txt"
sudo hwclock --show > "$RUN_DIR/rtc.txt" 2>&1
sudo systemctl cat grampusdrone.service --no-pager > "$RUN_DIR/service.txt"
sudo systemctl show grampusdrone.service --no-pager \
  -p Environment -p EnvironmentFiles -p NRestarts -p ActiveEnterTimestamp \
  > "$RUN_DIR/service-state.txt"
rosparam dump "$RUN_DIR/rosparams.yaml"
df -h "$HOME"
rosbag record -O "$RUN_DIR/ros.bag" \
  /fast_lio/odometry_full /mavros/odometry/out \
  /lio_to_mavros/healthy /lio_to_mavros/status /system_time/healthy \
  /mavros/state /mavros/local_position/odom /mavros/imu/data \
  /mavros/global_position/global /mavros/global_position/raw/fix \
  /mavros/timesync_status /rosout_agg
```

启动前确认核心话题存在；可选 GNSS/timesync 话题因配置不存在时记录此限制。首次记录不加入大点云，避免额外 I/O 干扰。留足空间，在试飞前地面确认记录没有明显拖慢系统。断热点测试用 Orin 本地终端运行记录，不能依赖会随热点断开的 SSH；上面的 rosbag 不会自动跨断电继续，冷启动每轮重新记录。

完成后 Ctrl+C 停止 rosbag，确认退出后再关机：

```bash
date -Is > "$RUN_DIR/end-time.txt"
sudo journalctl -u grampusdrone.service -b --no-pager > "$RUN_DIR/service-journal.txt"
sudo systemctl show grampusdrone.service --no-pager -p NRestarts \
  > "$RUN_DIR/restarts-after.txt"
rosbag info "$RUN_DIR/ros.bag" > "$RUN_DIR/bag-info.txt"
```

每次切换热点、开始搬运、解锁/起飞/降落、发现异常，观察员记录当地时间及事件。保存 QGC 参数文件、控制台文本和对应 PX4 ULog 到同目录，注明 EKF 主实例。日志可能包含位置/网络配置，保存在机载电脑或专用存储，不提交到 GitHub。

### 3.5 飞控日志与读数方法

QGC Console 逐条执行，等输出结束再执行下一条：

```text
logger status
listener estimator_selector_status 1
listener vehicle_visual_odometry 5
listener estimator_status_flags 5
listener estimator_aid_src_ev_pos 5
listener estimator_aid_src_ev_hgt 5
listener estimator_aid_src_ev_yaw 5
listener estimator_aid_src_ev_vel 5
listener vehicle_local_position 1
```

地面测试需要 ULog 时，先确认日志确实在录。若 logger 仅解锁后记录，可在未解锁地面用本机 `logger help` 核对后启动记录（常用 `logger on`），再次 `logger status` 确认，结束后按本机帮助停止；不可为了日志而解锁。ULog 的实际主题配置需核对，至少含所测辅助源、状态、局部位置和姿态；缺项则补齐记录再做故障/过渡试验，不能仅凭截图判定整个过程正常。

读数规则：

- 外部测量应新鲜、时间连续；辅助源应新鲜且有更新的 `time_last_fuse`。陈旧消息中的 `fused=True` 不算当前融合。
- 查看 selector 的主实例，分析该实例的状态和辅助源；也检查其他实例异常和主实例切换。两个实例不代表测了两架飞机。
- 只要求**本次配置期望融合的源**满足条件。GNSS 禁用时其 flags 为 False 正常；多源方案可能有意停止 EV yaw，不要求所有 EV flag 永远 True。
- `pose_frame=2` 为固定任意航向的局部 FRD 参考；`velocity_frame=3` 为机体 FRD。`cs_yaw_align=False` 在纯 EV 局部航向模式下可与 EV yaw 融合同时存在，但不是全球航向正确的证据。
- ROS 四元数打印顺序通常为 x/y/z/w；PX4 为 w/x/y/z。不要逐元素直接比较。起始 yaw 约 90° 也不是自动判错依据。
- 判断拒绝用 `innovation_rejected`、`reject_*`、原始 `test_ratio` 及时间序列。`test_ratio` 持续超过 1 需排查；不要将带符号滤波值的负数当作非法方差。
- `innovation_fault_status_changes` 是历史累计变化数，记录前后增量和发生时间，不能把它直接当作当前故障。
- `reset_counter=0`、`quality=0`、`device_id=0` 不是健康证明。本链路还需看 `vehicle_local_position` 的 xy/z/vxy/vz/heading reset counters（以实际字段为准）、估计器事件和进程重启。
- EV 消息的采样/接收时间差不等于可直接填入 EKF2_EV_DELAY 的数值；辅助源约 190–200 ms 的延迟还含 EKF 融合时域。当天不凭单帧截图调延迟。
- EKF 观测方差可能被参数下限或模式替代；小创新不代表绝对精度很高，不据此临时减小噪声。

### 3.6 拆桨地面运动：先验证方向和参考连续

1. 静置 120 s，记录初始位置、姿态、速度、健康状态及本地位置重置计数。
2. 保持机头方向，依次向前、后、右、左缓慢搬动约 0.5–1 m，再抬高/降低约 0.3–0.5 m。每次停稳 10 s。
3. 保持空间足够，缓慢 yaw 转动约 ±30°；小幅 roll/pitch 检查后放回。不要快速甩动或转圈丢失视场。
4. 搬运前、过程中和停稳后均记录；停稳后再运行 100 帧检查。核对 PX4 局部位置、姿态没有非物理跳变。

速度方向表（机头和倾斜近似保持，按各自机体系读）：

| 动作 | ROS 输出 twist：FLU | PX4 visual odom velocity：BODY_FRD |
|---|---|---|
| 向前 | x 正 | x 正 |
| 向右 | y 负 | y 正 |
| 向上 | z 正 | z 负 |
| 俯视机头向右转 | angular.z 负 | angular_velocity.z 正 |

**位置表达在固定参考系，机头旋转后不能再要求“向前走就是 position.x 增大”。** 比较 ROS 与 PX4 的位置或两个估计器前，要明确固定旋转/原点；不能直接相减当误差，也不能把 PX4 融合结果回灌校正 FAST-LIO。原地绕 B 转动时 B 平移应较小，手持不完美绕中心产生的小平移要与杆臂错误区别。

通过：方向正确、停稳回零、达到 2.2 门槛、所有预期融合持续，无无法解释的重置。失败则停止后续飞行。

## 4. 测试 001：RTC、联网状态与纯外部视觉

“纯视觉”在本文指 PX4 不融合 GNSS 的外部定位试验。FAST-LIO 本身使用 LiDAR+内部 IMU；PX4 仍使用自身 IMU，并可能融合气压计/磁力计，不是禁用所有非视觉传感器。

### 001-A 联网地面基线

1. 地面未解锁，设置并记录纯 EV 配置；若以禁用 GNSS 融合作为隔离手段，按实际固件参数说明操作、保存并按要求重启。若断开硬件，仅断电时操作，先确认没有一起拔掉需要的罗盘。飞行中不插拔 GPS。
2. 热点连接，在允许地面校时的阶段完成时间同步。启动/重启定位整栈前完成会跳变的校时；不在飞行中执行 `date -s`、`hwclock --hctosys` 或 `chronyc makestep`。
3. 运行 000 基线。确认 GNSS 位置、速度、高度辅助源未参与预期纯 EV 配置，不只看 GPS 插头。EV yaw 是否融合以已记录配置为准。
4. 静置 120 s 并做 000 地面运动，记录网络、时钟、ROS 和 PX4 状态。

### 001-B1 断热点但不重启：地面验证网络独立性

1. 先确认 RC 独立、记录在本地、QGC 有独立可用连接；否则不能把断热点当作纯网络变量测试。
2. 未解锁时断开热点，保持整栈运行 120 s，重复健康与频率检查。
3. 比较 `NRestarts`、FAST-LIO PID、时间戳、融合状态；不应因失去互联网而停止定位。
4. 需要测试重新联网时也只在地面：留存时钟和重启日志，待稳定并重新完成基线。重新联网校时引起的重启必须记录，不能当作无缝恢复通过。

通过仅表示“本次无网络运行不受影响”，不证明 RTC 能断电保持时间。

### 001-B2 三次冷启动：离线等待、联网校时后启动

每轮做以下完整过程，不把 warm reboot 当作断电试验：

1. 前一轮保存 rosbag/ULog，正常关闭 Orin，再按硬件规范切断主电源，保留设计中的 RTC 后备电源。
2. 确保所有互联网通路不可用（不只是手机热点），记录断电和重新上电时间。重新上电后不人工校时，不另外启动 ROS；用 Orin 本地终端观察至少 60 s。
3. 保存日期、RTC 和服务等待日志：

```bash
date -Is
timedatectl status
sudo hwclock --show
sudo systemctl status grampusdrone.service --no-pager -l
sudo systemctl show grampusdrone.service --no-pager -p NRestarts -p MainPID
sudo journalctl -u grampusdrone.service -b --no-pager
```

4. 预期没有本服务启动的 ROS 定位节点，日志显示等待网络，默认无限等待不增加重启计数。此时健康话题不存在是正常现象，不运行依赖 ROS 的 bag 记录或就绪脚本。
5. 开启并连接热点，不手动重启 grampusdrone。确认 timesyncd 本次开机同步成功后，经过至少 10 s 稳定窗口再启动定位；按 000 完成基线和 3.4 记录，静置 120 s。
6. 记录从联网到首次 READY 的时间和所有异常/重启。重复三轮，全部地面完成，不在等待结束后直接起飞。

通过门槛：离线不放行；联网且本次开机校时完成后自动 READY；无人工校时、无超时锁止、无非预期整栈/FAST-LIO 重启；正式输出和融合检查通过。初次校时必须早于 ROS 定位启动，不以“故障后重启恢复”代替正常启动通过。

RTC 的断电保时可继续单独记录、与可信时钟比较，但不再作为离线启动定位的依据。启动时从磁盘时间缓存恢复日期不算 RTC 保时通过。若连接的是无互联网的热点或 NTP 不可达，持续等待符合策略，不能跳过门槛。

### 001-C 首次纯 EV 悬停

前提：000、001-A/B 通过；地面运动和参考原点已确认；空间足够。天气/场地不合适可改期。起飞前保持联网并完成启动门槛，整个架次不主动切换网络。

1. 装桨前完成检查与记录。飞手确认 Position 可用及备用模式开关，观察员启动已验证的记录。
2. 每次上电后先等待初始化和 120 s 检查窗口，不自动起飞。
3. 人工起飞至预定短悬停高度，保持 20–30 s，按 2.2 观察漂移/振荡/异常。
4. 正常则降落、未解锁，检查 ULog 的振动、估计器拒绝/重置、位置/速度创新。第一架次不连做室内外切换。
5. 首架次通过后第二架次再做：短悬停 → 前后/左右小幅移动并停住 → 小角度 yaw → 降落。一次只做一个动作，不快速转向。
6. 本次不安排离线冷启动飞行。启动后的断网影响先按 001-B1 在地面验证；不在空中主动断热点。

记录实际高度、范围、时长、目视偏移和接管次数。任何异常中止后不得把“最终落地成功”写为通过。

## 5. 测试 002：多源定位及室内外过渡

### 5.1 必须先解决的航向策略

当前 MAVROS 发送 `LOCAL_FRD`，FAST-LIO world 的航向不保证与真北对齐。PX4 v1.17 的实现中，GNSS 已融合且 yaw 已对齐时，会停止不属于 LOCAL_NED 的 EV yaw 融合。因此“纯 EV 时正常”不能证明“EV yaw 与 GPS 一起长期正常”。

开始 002 前，在记录表明确写出：

- 纯 EV 时航向来源是什么；室外 GNSS 接入后航向来源是什么。
- GNSS 消失时是否仍有可用的航向/高度来源。
- 哪些阶段期望 EV yaw 开启/停止，是否期望某次受控的参考对齐；不允许现场凭猜测解释突跳。
- 磁力计在室内外是否可用；GNSS 接收机有无独立航向能力。普通单天线 GPS 的位置解不等于静止航向解。

可考虑经验证的独立航向方案，并按实际固件决定是否融合 EV yaw；若使用绝对航向 EV，需先完成参考系和航向标定。**不在当天凭名称把 frame 改为 NED，不只修改 frame_id，不直接复制 EV_CTRL/GPS_CTRL 掩码。** 未确定策略时，仅采集地面数据，002 飞行和 003 GNSS 接管都暂停。

### 002-A 室外 GNSS 独立可用性与多源地面基线

1. 落地未解锁，保存纯 EV 参数基线；按已确认的多源配置操作，按参数要求重启。物理 GPS 连接只在断电时改变。
2. 室外有 GNSS 视野且有充分 LiDAR 特征的场地静置。接收机有 fix/卫星数足够不是融合证据。
3. 保存 GNSS 信息和辅助源：

```text
listener sensor_gps 1
listener estimator_selector_status 1
listener estimator_status_flags 5
listener estimator_aid_src_gnss_pos 5
listener estimator_aid_src_gnss_vel 5
listener estimator_aid_src_gnss_hgt 5
listener vehicle_local_position 1
```

若固件话题名不同，运行 `uorb top` 查找本机 GPS/GNSS 和 aid source 名称；缺辅助源时用 ULog 等效验证，不把无输出当通过。高度源按配置检查，不要求 GNSS 高度一定融合。

4. 确认主 EKF 实例的 GNSS 水平位置/预期速度正在融合，EV 各源符合预先策略；姿态/航向已建立，局部位置/速度有效，未处于水平惯性推算。
5. 重复 000 地面运动。GNSS/EV 位置、速度、航向均无持续拒绝；位置参考和实际运动一致。静置至少 120 s。
6. GNSS 独立维持能力在第 6 节的拆桨断流试验中单独验证，不能仅因它与 EV 一起 fused 就认定失去 EV 也能维持。

### 002-B 拆桨搬运室内 → 室外 → 室内

1. 全程未解锁且记录连续；从已稳定场景出发，保持机头方向缓慢搬运。
2. 在门口两侧分别停留约 30–60 s，走到目标场景后停留至少 120 s；观察 GNSS 实际取得/丢失与 EKF 状态变化，不仅看是否穿过门。
3. 再反向搬运。记录每次 yaw 对齐变化、EV yaw 是否停止/恢复、局部状态 reset counters、selector 主实例和辅助源实际融合。
4. 一次双向通过后，视余量重复两次。大幅位置/航向跳变、参考无法解释、GNSS 仅间歇拒绝、桥接锁止，均禁止进入过渡飞行。

地面正常不是飞行证明，但地面失败必须先修复。没有宽阔通道、可中途降落区或持续 LiDAR 特征时，取消穿越飞行，仅做两端独立测试。

### 002-C 室外多源短悬停

满足 002-A/B、001-C 和航向策略确认后执行。固定网络条件；先悬停 20–30 s、落地检查，再做小幅运动。GNSS/EV 状态须符合本架次预期，不要求所有源一直开启；任何无法解释的模式/航向变化中止。未完成独立 GNSS 保持检查时，不把本架次当作 GNSS 后备试验。

### 002-D 可选低速双向过渡飞行

只有两端各自悬停、地面双向过渡，以及第 6 节 GNSS 独立保持地面试验通过并完成地面恢复后才考虑，允许当天不执行。第 6 节可提前在 002-A 之后执行；章节顺序不是跳过此门槛的理由。

1. 单独架次只做一个方向，起点和终点均有充足起降空间。避开狭窄门洞、低天花板和人员。
2. 起点稳定短悬停后，以预定低速通过宽阔过渡区，不同时升高、急转或改变网络。
3. 目标区短停观察，条件允许则落地；查日志后才能安排反方向下一架次。
4. 先做一组并评估，不默认连续三次。只有前一组记录通过才允许重复。

通过需同时满足：可控且无明显跳变、实际融合按预期变化、局部状态有效、没有无法解释的重置/拒绝或服务重启。飞机仍显示 Position 不能独立作为通过标准。

## 6. 测试 003：外部测量断流与 GNSS 保持（仅地面）

目的：区分“外部输入消失后 GNSS 能否独立维持”与“FAST-LIO 地图错误”；本项只验证前者。**拆桨、未解锁、室外、GNSS 已实际融合且航向来源不依赖即将消失的 EV**。全程不重新解锁。

不使用升高高度、遮挡雷达、拔线或杀死整个服务制造故障。停止整个服务会一起断开 MAVROS，无法隔离验证 EV 断流。

步骤：

1. 完成 002-A；记录 GNSS 和 EV 辅助源、局部状态及 120 s 基线，确保地面 ULog 和 rosbag 在记录。
2. 再确认 `/mavros/state` 为 `connected=True, armed=False`。在 Ubuntu **仅停止桥接节点**：

```bash
rosnode kill /lio_to_mavros
```

3. 当前 bridge launch 不 respawn，预期桥接进程停止，FAST-LIO 和 MAVROS 继续运行：

```bash
rosnode list
rostopic info /mavros/odometry/out
timeout 10s rostopic hz /mavros/odometry/out
timeout 5s rostopic echo -n 1 /mavros/state
```

4. 在 PX4 侧连续观察至少 30 s；按固件超时 EV flags 应退出，旧的 EV topic 可保留最后一条，但其年龄增长、`time_last_fuse` 不再前进。不要读陈旧 `fused=True` 误判。
5. 检查主实例 GNSS 辅助源仍新鲜融合，航向和局部位置/速度有效，没有进入水平惯性推算。记录模式、航向/位置 reset、状态变化；若无 EV 后 GNSS 或 yaw 也失效，本项失败。
6. 保存数据。地面停止服务、按第 8 节恢复正式模式并重做初始化；不要只重新启动桥接器然后认为参考自动衔接。

本项测的是“输出链路停止”，不会产生桥接器自身 FAULT，因为桥接器已退出。不要把没有 FAULT 消息当作检测失败。后续第 7 节单独验证上游断流时仍存活的桥接器如何锁止。

通过仍不授权空中停节点：拆桨静止没有验证解锁状态下的故障响应、控制瞬态和地面效应。本次不执行空中注入。

## 7. 测试 004：FAST-LIO 重启与地面恢复（不制造真实发散）

本项验证“上游退出/重启后拒绝静默恢复”的边界，不测试地图发散自动识别、不验证新旧地图无缝连接。拆桨、未解锁并全程记录；先完成 003 并恢复健康基线。

1. 确认桥接器 READY、MAVROS 连通，记录 `/laserMapping` PID、局部状态 reset counters、systemd `NRestarts`。
2. 仅在上述地面条件满足时停止 FAST-LIO 节点：

```bash
rosnode info /laserMapping
rosnode kill /laserMapping
```

3. 当前 launch 配置预计约 2 s 后重新拉起 FAST-LIO；0.5 s 输入超时应使桥接器锁止。观察：

```bash
timeout 5s rostopic echo -n 1 /lio_to_mavros/status
timeout 5s rostopic echo -n 1 /lio_to_mavros/healthy
timeout 10s rostopic hz /mavros/odometry/out
rosnode info /laserMapping
sudo journalctl -u grampusdrone.service -b -n 120 --no-pager
```

4. 预期 bridge 为 FAULT、healthy=False，正式 odometry 停止。即使 FAST-LIO 新 PID 再次输出，新数据也不应使已锁止桥接器自动恢复。ROS 仍显示 odometry publisher 不代表还有消息。
5. PX4 EV 旧数据应变陈旧并退出融合；GNSS 是否维持按第 6 节评估。`reset_counter` 可能仍为 0，不能用它判断这次 FAST-LIO 重启是否发生。
6. 保存故障数据，落地未解锁状态按第 8 节重启整栈，等待静止初始化，重新记录新基线。恢复可能建立新参考，不承诺原地图坐标或航迹连续。
7. 第一轮完全符合预期后，最多在地面独立重复三轮。每轮单独保存故障前/中/后日志，不与室内外过渡叠加。

如果桥接器在上游重启后无提示恢复、输出仍有异常数据，或 systemd 连带重启造成状态与预期不符，记录为失败，停止飞行试验并分析实际源码/版本。此时不要靠提高超时或关闭检测使其“通过”。

## 8. 地面恢复与回退

### 8.1 故障后恢复正式 odometry

飞机落地、未解锁，故障日志已保存，确认没有并行手动 launch：

```bash
sudo systemctl stop grampusdrone.service
ps -ef | grep -E 'roslaunch.*onboard_stack|lio_to_mavros_node|fastlio_mapping' | grep -v grep
```

预期旧进程消失，有残留则查明所属启动方式，不盲目 kill 全部进程。确认 drop-in 正式配置：

```bash
sudo tee /etc/systemd/system/grampusdrone.service.d/lio-output.conf >/dev/null <<'EOF'
[Service]
Environment=LIO_OUTPUT_MODE=odometry
Environment=LIO_ODOMETRY_PREVIEW=false
Environment=START_CONTROLLER=false
Environment=START_PLANNER=false
EOF
sudo systemctl daemon-reload
sudo systemctl reset-failed grampusdrone.service
sudo systemctl start grampusdrone.service
```

保持飞机静止，等待时间门槛及 FAST-LIO/桥接初始化，重做 000 健康、同时间戳检查及 PX4 融合检查。不能仅看到 active 就马上起飞。空中不执行本节。

### 8.2 回退旧 vision_pose

回退用于结束新协议测试；旧 pose 的测量原点、时间处理、速度输入和完整 odometry 不同。若飞控参数曾为新模式修改，必须恢复**与旧链路对应**的已验证配置；尤其已开启 EV 速度融合时，不能认为 pose 接口也能提供相同速度观测。

```bash
sudo systemctl stop grampusdrone.service
sudo tee /etc/systemd/system/grampusdrone.service.d/lio-output.conf >/dev/null <<'EOF'
[Service]
Environment=LIO_OUTPUT_MODE=vision_pose
Environment=LIO_ODOMETRY_PREVIEW=false
Environment=START_CONTROLLER=false
Environment=START_PLANNER=false
EOF
```

在 QGC 恢复旧链路参数并按参数要求重启飞控，仍保持未解锁。然后：

```bash
sudo systemctl daemon-reload
sudo systemctl reset-failed grampusdrone.service
sudo systemctl start grampusdrone.service
rosparam get /lio_to_mavros/output_mode
timeout 5s rostopic echo -n 1 /lio_to_mavros/healthy
rostopic info /mavros/odometry/out
timeout 10s rostopic hz /mavros/vision_pose/pose
cd ~/GrampusDrone
REQUIRE_SYSTEM_TIME_MONITOR=true bash shfiles/wait_for_stack.sh 120
```

确认 vision_pose、健康 True、旧 pose 约 30 Hz、正式 odometry 无桥接器发布者、PX4 实际融合符合旧配置。回退不需要重置 Git，不代表未经检查可立即再飞。旧模式没有根 status 话题是当前实现的正常行为。

## 9. 每项结果记录模板

每轮复制一份到本地测试目录。结论用“通过 / 失败 / 中止 / 未执行”，不能用空白代表通过。

```text
编号与子项：
实际日期/开始结束时间/boot-id：
飞手/观察员/场地/天气或室内条件：
Git commit / MAVROS / PX4 版本：
PX4 参数文件 / ROS 参数文件 / service 配置：
网络状态（是否影响 RC/QGC/RTK）：
GNSS 配置与实际融合：
航向来源及各阶段预期 flags：
高度来源：
B 与重心关系及 EV_POS/IMU_POS 核对：
RTC 与参考时钟偏差 / 上电至 READY 时间：
飞行前 bridge/time healthy / 平均频率 / 同时间戳检查：
systemd NRestarts 前后 / FAST-LIO PID 前后：
主 EKF 实例与切换次数：
输入 reset_counter 与 PX4 本地状态 reset counters 前后：
动作、时间、高度、范围、持续时间：
静止漂移 / 悬停偏差 / 运动方向 / 停稳速度：
EV/GNSS 各源拒绝、最大原始 test_ratio、异常时段：
飞行模式变化 / 人工接管 / 异常与处置：
rosbag / ULog / journal / 视频文件名：
结论及依据：
允许推进到哪一项（未解决问题则写停止）：
```

当日结束核对：

- [ ] 每架次和每个故障试验都有独立日志、参数和结论；成功与失败都保存。
- [ ] 对应主 EKF 的状态有效性、辅助源新鲜度、拒绝、reset 和振动已检查。
- [ ] 冷启动、联网条件、多源航向策略分别记录，没有混为“定位正常”。
- [ ] 留给下次上电的 systemd/PX4 配置明确；控制器/规划器仍关闭。
- [ ] 未执行高空退化或空中恢复，也不在结论中声称这些能力已具备。

## 10. 现场简表

1. **先不装桨**：保存版本/参数 → 原点与航向策略 → 独立接管 → 日志 → 000 地面运动。
2. 联网地面基线 → 断网地面运行 → 三次离线等待再联网冷启动，不直接起飞。
3. 纯 EV 首次短悬停 → 落地查日志 → 小幅运动；有问题当天停在此阶段。
4. 多源先地面确认 GNSS 真正融合与航向策略 → 拆桨双向搬运 → 条件满足再各场景短悬停。
5. 室内外过渡飞行是可选末项，必须前项通过；不同时升高、切网或注入故障。
6. 003/004 只在拆桨地面做；完成后重新建立基线，不从故障状态直接起飞。
7. 每项失败先落地、保存、分析；不现场放宽门槛、不在空中重启。

## 11. 相关文档与实现依据

- [完整里程计双模式及地面测试](lio_to_mavros_dual_output.md)
- [systemd 自启动说明](onboard_autostart.md)
- [FAST-LIO 完整里程计](fastlio_full_odometry.md)
- 本仓库：`src/realflight_modules/px4ctrl/launch/onboard_stack.launch`、`src/realflight_modules/FAST_LIO/launch/mapping_mid360.launch`、`src/realflight_modules/lio_to_mavros/launch/bridge.launch`、`shfiles/start_onboard.sh`、`shfiles/monitor_system_time.py`。
- [PX4 v1.17 外部视觉 yaw 控制与 GNSS 条件](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp)
- [PX4 v1.17 EV 参数定义](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/params_external_vision.yaml)
- [MAVROS 1.20.1 odometry 插件](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp)
