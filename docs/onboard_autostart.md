# Orin NX 上电自动启动

自动启动只让系统进入待飞状态，不会自动解锁或起飞。定位门控现在会在数据明显异常时停止向 MAVROS 发布视觉位姿；原来的 ready_go.sh 和 PX4 参数未改动。

## 新增文件

- src/realflight_modules/px4ctrl/launch/onboard_stack.launch：统一启动 Livox、MAVROS、FAST-LIO 和视觉里程计转发，可选启动控制器和规划器。
- shfiles/wait_for_stack.sh：检查 MAVROS 连接和关键定位话题。
- shfiles/start_onboard.sh：供 systemd 调用的启动入口。
- shfiles/grampusdrone.service：systemd 服务模板。
- shfiles/wait_for_stable_clock.py：ROS 启动前的系统时间门槛。
- shfiles/monitor_system_time.py：运行期间的系统时间跳变监视器。

## 系统时间门槛

已安装旧版服务的 Ubuntu 设备，在获取本次代码后，先在飞机未解锁的地面状态执行：

~~~bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
catkin_make -j$(nproc)
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh
sudo cp shfiles/grampusdrone.service /etc/systemd/system/grampusdrone.service
sudo systemctl daemon-reload
sudo systemctl start grampusdrone.service
journalctl -u grampusdrone.service -b --no-pager
~~~

已有的 service override 会继续生效；首次测试应确认控制器和规划器仍关闭。修改源码后需要重新编译，否则旧转发节点不会检测系统时间跳变。

systemd 启动 ROS 前会运行 `wait_for_stable_clock.py`，必须同时满足以下条件：

1. 系统时间不早于 `2025-01-01 00:00:00`（`MIN_VALID_EPOCH=1735689600`），用于拦截没有有效 RTC 时常见的 1970 年时间。
2. 连续 10 秒内，实时时钟相对于单调时钟的变化差不超过 0.5 秒。平滑校时可以通过；从 1970 年突然跳到当前日期会重新计时，不会在跳变期间启动 ROS。

这只能证明时间没有正在跳变，不能证明日期绝对正确。若设备离线且时间仍是 1970 年，服务进程会一直等待，不会启动定位节点；由于服务使用 `Type=simple`，`systemctl status` 仍可能显示 `active (running)`，需查看日志确认。先检查：

~~~bash
timedatectl
date
sudo hwclock --show
~~~

确认系统时间正确后，可以尝试写入硬件 RTC：

~~~bash
sudo hwclock --systohc
~~~

只有 Orin 的 RTC 硬件和后备电源可用时，完全断电后才会保留时间；需要断电重启后再次用 `date` 和 `sudo hwclock --show` 验证。不要为了绕过等待而把 `MIN_VALID_EPOCH` 改成 1970 年。

ROS 启动后，`monitor_system_time.py` 继续监视时间跳变：

- MAVROS 状态新鲜、显示已连接 PX4 且未解锁：监视器终止 `start_onboard.sh`，systemd 自动重启整套 ROS；重启前会再次经过时间门槛。
- PX4 已解锁或状态未知：不自动重启；`lio_to_mavros` 会关闭视觉位姿转发并记录错误。不要在飞行中重启定位。

检查时间监视器：

~~~bash
rostopic echo -n 1 /system_time/healthy
journalctl -u grampusdrone.service -b --no-pager
~~~

## FAST-LIO 里程计保护（待在 Orin NX 上验证）

`onboard_stack.launch` 中的 `lio_to_mavros` 先等待 `/Odometry` 连续静止 5 秒，期间不发布 `/mavros/vision_pose/pose`。飞机在上电后搬动时，它会继续等待；放稳后才建立相对零点。`/lio_to_mavros/healthy` 为 `true` 表示门控已开放，`false` 表示禁止转发。`wait_for_stack.sh` 也会要求它为 `true`。

发生 NaN/Inf、位置超过 1000 m、速度超过 20 m/s、姿态四元数不合理、位置突跳、时间戳回退或与系统时钟相差超过 2 秒，以及已开放后里程计超过 0.5 秒没有更新，均停止视觉转发。异常日志可用 `rosnode info /lio_to_mavros` 和 `journalctl -u grampusdrone.service -b` 排查。阈值是保守的初始值，实际飞行前需根据地面数据确认。

只有**尚未向 MAVROS 发布过视觉位姿**、MAVROS 状态新鲜且 PX4 明确未解锁时，数值异常或突跳才可能自动重启 `/laserMapping`；`roslaunch` 会把该进程重新拉起，最多尝试 3 次。时间戳异常不靠重启处理。一旦视觉已经开始发布，或 PX4 曾经解锁，检测到故障就保持门控关闭，必须人工检查并在安全状态下重启整套服务。不要在飞行中重启定位。没有消息、缓慢漂移或 LiDAR 点云质量差不能仅靠这个门控判定为定位准确；当前时钟和 Livox 异常也仍需单独排查。

地面无桨检查：在飞机静止、飞控未解锁时启动 `onboard_stack.launch`；观察 `/lio_to_mavros/healthy` 从 `false` 变成 `true`，再观察 `/mavros/vision_pose/pose`。搬动飞机时应保持连续，切勿把此检查视为飞行许可。若要关闭预放行自动重启，可在 launch 的 `lio_to_mavros` 节点下把 `auto_restart_before_first_publish` 改为 `false`。

## 第一步：拉取和编译

在 Ubuntu 上执行：

~~~bash
cd ~/GrampusDrone
git pull --rebase origin master
catkin_make -j$(nproc)
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh
~~~

如果通过 Git 拉取后新增的 Python 脚本没有执行权限，在 Ubuntu 上补一次：

~~~bash
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh \
  shfiles/wait_for_stable_clock.py shfiles/monitor_system_time.py
~~~

## 第二步：只测试定位链路

新 launch 默认不启动 px4ctrl 和规划器：

~~~bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash
python3 shfiles/wait_for_stable_clock.py
roslaunch px4ctrl onboard_stack.launch fcu_url:=/dev/ttyACM0:57600
~~~

上面的手动 `roslaunch` 只会执行时间门槛，不会自动启动运行期时间监视器；完整的运行期时间监视只由 `grampusdrone.service` 提供。首次测试建议优先使用 systemd 服务，或者手动启动后不要在时间可能跳变时继续使用定位数据。

另开终端检查：

~~~bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash
bash shfiles/wait_for_stack.sh 120
~~~

检查通过后按 Ctrl+C 停止 launch。若失败，原来的命令仍可直接使用：

~~~bash
sh shfiles/ready_go.sh
~~~

## 第三步：手动测试完整待飞链路

确认桨叶已经拆除，遥控器可随时接管，然后执行：

~~~bash
roslaunch px4ctrl onboard_stack.launch \
  fcu_url:=/dev/ttyACM0:57600 \
  start_controller:=true \
  start_planner:=true
~~~

该命令不会主动发布起飞指令。确认 MAVROS、里程计、EKF2 和遥控器状态正常后，按 Ctrl+C 停止。

## 第四步：安装 systemd 服务

只有前两次手动测试都正常时才执行：

~~~bash
cd ~/GrampusDrone
sudo cp shfiles/grampusdrone.service /etc/systemd/system/grampusdrone.service
sudo systemctl daemon-reload
sudo systemctl enable grampusdrone.service
sudo systemctl start grampusdrone.service
~~~

服务模板第一次安装时只自动启动定位链路，控制器和规划器默认关闭。检查服务：

~~~bash
systemctl status grampusdrone.service
journalctl -u grampusdrone.service -f
~~~

定位服务经过多次重启测试且状态稳定后，确认桨叶已经拆除，再创建一个独立的 systemd override 启用控制器和规划器：

~~~bash
sudo systemctl edit grampusdrone.service
~~~

在编辑器中写入并保存：

~~~ini
[Service]
Environment=START_CONTROLLER=true
Environment=START_PLANNER=true
~~~

然后应用并测试：

~~~bash
sudo systemctl daemon-reload
sudo systemctl restart grampusdrone.service
journalctl -u grampusdrone.service -f
~~~

日志正常后再重启测试：

~~~bash
sudo reboot
~~~

重启后查看本次开机日志：

~~~bash
systemctl status grampusdrone.service
journalctl -u grampusdrone.service -b --no-pager
~~~

## 常见启动失败

### 1. `status=203/EXEC` 或 `Permission denied`

典型日志：

~~~text
Failed at step EXEC spawning /home/orin/GrampusDrone/shfiles/start_onboard.sh: Permission denied
grampusdrone.service: Main process exited, status=203/EXEC
~~~

这表示 systemd 已经找到启动脚本，但脚本没有执行权限。检查权限：

~~~bash
cd ~/GrampusDrone
ls -l shfiles/start_onboard.sh shfiles/wait_for_stack.sh
~~~

权限中应包含 `x`，例如 `-rwxr-xr-x`。若没有，执行：

~~~bash
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh
sudo systemctl restart grampusdrone.service
~~~

仓库已经保存了这两个脚本的可执行位；但通过 ZIP、Windows 文件系统或其他不保留 Unix 权限的方式复制工程时，仍可能需要重新执行 `chmod +x`。

### 2. `ROS_DISTRO：未绑定的变量`

典型日志：

~~~text
/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: ROS_DISTRO：未绑定的变量
~~~

旧版 `start_onboard.sh` 在加载 ROS 环境前执行了 `set -Eeuo pipefail`。其中 `-u` 会让 Bash 在 ROS 环境脚本读取尚未定义的变量时立即退出。当前脚本已改为：

~~~bash
set -Eeo pipefail
~~~

如果使用的是旧版本工程，可删除该行中的 `u`，然后重启服务：

~~~bash
sudo systemctl restart grampusdrone.service
~~~

排查时只查看本次开机的最近日志，避免把修复前的旧错误误认为当前错误：

~~~bash
systemctl status grampusdrone.service --no-pager -l
sudo journalctl -u grampusdrone.service -b -n 100 --no-pager
~~~

## 回退

出现任何异常时，停用服务即可恢复原来的手动启动方式：

~~~bash
sudo systemctl disable --now grampusdrone.service
~~~

若只是想关闭控制器和规划器、保留定位链路，可删除 override：

~~~bash
sudo rm -f /etc/systemd/system/grampusdrone.service.d/override.conf
sudo systemctl daemon-reload
sudo systemctl restart grampusdrone.service
~~~

完全删除 systemd 安装项：

~~~bash
sudo rm /etc/systemd/system/grampusdrone.service
sudo systemctl daemon-reload
~~~

如果还没有安装 systemd 服务，只需按 Ctrl+C 停止新 launch，然后继续使用 ready_go.sh，不需要执行 Git 回退。

如果需要撤销 GitHub 上这次新增文件的提交，先找到提交号，再创建反向提交：

~~~bash
cd ~/GrampusDrone
git log --oneline -5
git revert <本次提交号>
git push origin master
~~~

不要使用 git reset --hard 回退已经推送的提交。

## 就绪检查内容

检查脚本确认 MAVROS 的 connected=true，并要求以下话题连续两次收到消息：

~~~text
/livox/lidar
/livox/imu
/Odometry
/lio_to_mavros/healthy (data: true)
/mavros/vision_pose/pose
/mavros/local_position/odom
~~~

它还保留了原 ready_go.sh 中的两条 PX4 消息频率请求。检查失败时，systemd 会终止并在 5 秒后重新尝试；整个流程不会发布解锁或起飞命令。
