# 将 FAST-LIO 点云转换到 PX4 本地坐标系（第一阶段：地面测试）

新增包：`src/realflight_modules/lio_cloud_to_mavros`。本阶段只新增转换节点、配置和测试，**不改变 FAST-LIO、视觉转发、EGO、px4ctrl 或 systemd 的启动行为**。不会解锁、切换飞行模式或起飞。

## 输入、输出与外参

| 用途 | 话题 | 坐标/时间约定 |
| --- | --- | --- |
| 输入扫描点云 | `/cloud_registered` | FAST-LIO world；当前扫描，不是累计地图 |
| 还原雷达坐标所需位姿 | `/Odometry` | IMU 在 world 中的位姿；必须与点云时间戳完全相同 |
| 投影所需位姿 | `/mavros/local_position/odom` | base_link 在 PX4 local ENU 中的位姿；按点云时刻插值 |
| 输出扫描点云 | `/cloud_registered_mavros` | 使用 MAVROS 里程计实际的 `header.frame_id`，不重写采样时间 |
| 转换状态 | `/registered_cloud_to_mavros/healthy`、`/registered_cloud_to_mavros/status` | 只说明转换依赖和时间匹配状态，不是飞行安全/EKF 健康结论 |

本节点不订阅 `/cloud_registered_body`。但需要澄清：这个话题本身并不会必然产生偏移，只要 IMU 到机身的外参正确也能使用。本节点采用 world 点云，再利用已有雷达—IMU 外参恢复到雷达原点，满足当前的安装参数表达方式。

全部可调项在：

```text
src/realflight_modules/lio_cloud_to_mavros/config/registered_cloud_to_mavros.yaml
```

主要安装参数：

```yaml
lidar_in_base_link:
  translation: [0.0, 0.0, 0.07]
  rpy_degrees: [0.0, 0.0, 0.0]
```

这表示 **雷达数据坐标原点在 MAVROS base_link 中的位置**，单位米，轴为 x 前、y 左、z 上。旋转为雷达轴到 base_link 轴的固定轴 RPY，单位度，顺序 `Rz(yaw) * Ry(pitch) * Rx(roll)`。例如移到机身前方 5 cm、高 7 cm，就改为 `[0.05, 0.0, 0.07]`。

**外壳几何中心、雷达数据坐标原点、内置 IMU 原点可能不是同一点。** 当前默认按你提供的水平居中、上方 7 cm，暂时视为雷达数据原点的位置。若测的是外壳中心，需要根据 MID360 厂商坐标定义补偿中心到数据原点的偏移。也要确认 base_link 对应的飞行器参考点确实是你测量的中心。这些不能由软件自动确认。

雷达到 IMU 的另一组外参自动读取运行中的 `/mapping/extrinsic_R`、`/mapping/extrinsic_T`，方向与 FAST-LIO 相同：

```text
p_IMU = R_IMU_LiDAR * p_LiDAR + t_IMU_LiDAR
```

当前工程的 `extrinsic_T` 为 `[-0.011, -0.02329, 0.04412]`，本节点不会重复硬编码这些数值，不能直接把它当作机身安装外参。若 FAST-LIO 位于其他 namespace，调整 `fastlio_mapping_namespace`。要求 `/mapping/extrinsic_est_en=false`，因为在线更新的外参不包含在 `/Odometry` 中，无法准确逆变换。

## 实际转换

记 W=FAST-LIO world、I=雷达 IMU、L=雷达数据坐标、B=base_link、M=PX4 本地坐标：

```text
p_L = R_ILᵀ * (R_WIᵀ * (p_W - t_WI) - t_IL)
p_B = R_BL * p_L + t_BL
p_M = R_MB * p_B + t_MB
```

所有变换使用完整三维旋转。取对应扫描时刻的两个位姿，不假设 world 到 PX4 local 只有固定平移。XYZ 和存在的 normal_x/y/z 会转换，强度、其他字段、行填充及源时间戳保留；不发布 TF，不给 PX4 发送新的视觉观测。

原始 world 点云和 `/Odometry` 来自同一次 FAST-LIO 更新，因此严格按相同时间戳配对；MAVROS 位姿在两条消息之间线性插值位置、SLERP 插值姿态。不外推，不用最新位姿代替采样时刻。队列和缓存有上限；等待超时、过旧点云、位姿采样间隔过大时不输出对应扫描。

## Ubuntu 上同步、编译

先拆桨/确保不会解锁，检查工作区没有需要保留的未提交修改，再执行：

```bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
git status
git pull --rebase origin master

source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
python3 -c 'import numpy; print(numpy.__version__)'
# 仅当上面的 import 报缺少 numpy 时安装：
# sudo apt install python3-numpy

catkin_make -j$(nproc)
source ~/GrampusDrone/devel/setup.bash
```

如果 `git status` 有源码修改，不要使用 `reset --hard` 或盲目丢弃它们。文件权限变化和日志变化要分别检查；本新增包不要求手动执行 `chmod +x`，catkin 会生成 Python 节点的可执行入口。

可先运行不依赖 ROS 节点的数学/点云格式测试：

```bash
python3 -m unittest discover \
  -s src/realflight_modules/lio_cloud_to_mavros/test -p 'test_*.py' -v
```

## Ubuntu 上地面测试

1. 启动现有定位服务，确认控制器、规划器仍关闭。如果之前开启过它们，先查看服务/override，或者停止服务，使用下面的手动定位 launch。**二选一，不要重复启动同一链路。**

```bash
# 使用服务（先确认服务参数 start_controller=false、start_planner=false）
systemctl cat grampusdrone.service
# 确认模板和 override 均未启用控制器/规划器之后再启动：
sudo systemctl start grampusdrone.service
rosnode list

# 或者：停止服务后在另一个终端保持运行
# sudo systemctl stop grampusdrone.service
# roslaunch px4ctrl onboard_stack.launch start_controller:=false start_planner:=false
```

2. 检查原始输入与外参。节点默认要求 MAVROS connected 状态新鲜、现有视觉保护为 true。

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /lio_to_mavros/healthy
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/child_frame_id
rostopic echo -n 1 /mavros/local_position/odom/header
rostopic echo -n 1 /mavros/local_position/odom/child_frame_id
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
rosparam get /mapping/extrinsic_est_en
```

应为 LIO `world/body`、MAVROS 本地 frame（常见 `map`）/`base_link`，在线外参估计关闭。两种 world 不应使用相同 frame 名称，不能靠改点云字符串伪装对齐。

3. 新终端 source 环境，独立启动转换节点。

```bash
cd ~/GrampusDrone
source devel/setup.bash
roslaunch lio_cloud_to_mavros registered_cloud_to_mavros.launch
```

4. 再开一个终端，检查输出。

```bash
source ~/GrampusDrone/devel/setup.bash
rostopic echo -n 1 /registered_cloud_to_mavros/status
rostopic echo -n 1 /registered_cloud_to_mavros/healthy
rostopic echo -n 1 /cloud_registered_mavros/header
rostopic hz --wall-time /mavros/local_position/odom
rostopic hz --wall-time /cloud_registered
rostopic hz --wall-time /cloud_registered_mavros
```

状态应为 READY、healthy=true。输出 header 使用实际 MAVROS 本地 frame，采样时间与对应输入扫描一致；输出率可能低于输入率，但不应长期无输出。先观察实际频率再调阈值，不要为获得输出而关闭时间检查。原始点云异常高频时，先检查雷达驱动；本节点不会修复上游频率问题。

5. 打开 RViz，把 Fixed Frame 设为输出点云实际 frame（通常 map），添加 `/cloud_registered_mavros` 的 PointCloud2 和 `/mavros/local_position/odom` 的 Odometry。此阶段不直接在旧 EGO RViz 配置中点击目标。对着固定墙面，静止观察，再地面缓慢平移、转动机体；点云与机体位置应一致，墙不应随机体转动或出现明显重影。检查高度时须注意 PX4 local 的 z=0 不一定就是地面。

如要使用不同安装配置，在仓库外复制 YAML 后启动，避免阻碍下次 pull：

```bash
mkdir -p ~/grampusdrone-config
cp src/realflight_modules/lio_cloud_to_mavros/config/registered_cloud_to_mavros.yaml \
   ~/grampusdrone-config/registered_cloud_to_mavros.yaml
gedit ~/grampusdrone-config/registered_cloud_to_mavros.yaml
roslaunch lio_cloud_to_mavros registered_cloud_to_mavros.launch \
  config:=$HOME/grampusdrone-config/registered_cloud_to_mavros.yaml
```

参数在启动时读取，修改安装参数后 Ctrl+C 并重新启动转换节点；不要在飞行中改外参。后续更换雷达，还需重新标定 FAST-LIO 的雷达—IMU 外参。

## 无输出/FAULT 与回退

- `WAITING: ... extrinsics`：未运行 FAST-LIO、namespace 不对或缺少外参。
- `WAITING: ... connected/healthy/odometry`：对应输入缺失、不新鲜或视觉保护尚未放行。
- `WAITING: matched ...` / dropped：没有同时间戳 LIO 位姿、没有覆盖扫描时刻的 MAVROS 位姿或时间差超限。先检查时间戳与输入速率，不能仅以 `rostopic hz` 正常判断同步正确。
- `FAULT`：坐标名不符、系统时间跳变、位姿明显跳变、输入重置/中断后恢复或外参变化等。错误锁存，不自动重新开始输出，也不自动重启 FAST-LIO/PX4。地面停止并检查原因，稳定后手动重启转换 launch。

此阶段回退只需在转换终端按 Ctrl+C，输出话题停止；现有定位、视觉转发和旧规划启动保持不变，无需 Git 回退或安装新的 systemd 服务。

**尚未迁移 EGO。** 它仍订阅 `/imu_propagate` 与 `/cloud_registered`。后续应一起切换里程计、转换后点云、地图/轨迹 frame 和 RViz 目标参考系，同时增加估计重置与超时保护。GPS/EKF 校正可能造成投影点云和旧地图不一致；本节点只做明显跳变检测，不保证检测所有 PX4 重置，不会自动清空 EGO 地图，也不是飞行许可。

当前视觉转发的原点/姿态对齐、PX4 融合配置和控制器的速度处理仍需独立检查。没有完成这些地面验证前，不应将转换成功视为可以直接自动飞行。
