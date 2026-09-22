# Orin NX 上电自动启动

本方案只增加文件，不修改原来的 ready_go.sh、控制源码或 PX4 参数。自动启动只让系统进入待飞状态，不会自动解锁或起飞。

## 新增文件

- src/realflight_modules/px4ctrl/launch/onboard_stack.launch：统一启动 Livox、MAVROS、FAST-LIO 和视觉里程计转发，可选启动控制器和规划器。
- shfiles/wait_for_stack.sh：检查 MAVROS 连接和关键定位话题。
- shfiles/start_onboard.sh：供 systemd 调用的启动入口。
- shfiles/grampusdrone.service：systemd 服务模板。

## 第一步：拉取和编译

在 Ubuntu 上执行：

~~~bash
cd ~/GrampusDrone
git pull --rebase origin master
catkin_make -j$(nproc)
chmod +x shfiles/start_onboard.sh shfiles/wait_for_stack.sh
~~~

## 第二步：只测试定位链路

新 launch 默认不启动 px4ctrl 和规划器：

~~~bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash
roslaunch px4ctrl onboard_stack.launch fcu_url:=/dev/ttyACM0:57600
~~~

另开终端检查：

~~~bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash
sh shfiles/wait_for_stack.sh 120
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
/mavros/vision_pose/pose
/mavros/local_position/odom
~~~

它还保留了原 ready_go.sh 中的两条 PX4 消息频率请求。检查失败时，systemd 会终止并在 5 秒后重新尝试；整个流程不会发布解锁或起飞命令。
