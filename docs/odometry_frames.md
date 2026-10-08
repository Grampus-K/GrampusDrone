# FAST-LIO 里程计坐标约定

本次统一线速度语义，不切换 MAVROS 输入方式：目前仍通过 `/mavros/vision_pose/pose` 发送位姿。

## 两个输出话题

| 字段 | `/Odometry` | `/imu_propagate` |
| --- | --- | --- |
| 更新来源 | LiDAR 更新后的滤波状态 | IMU 外推状态 |
| `header.frame_id` | `world` | `world` |
| `child_frame_id` | `body` | `body` |
| `pose.pose.position` | IMU 原点在 world 中的位置 | 同左 |
| `pose.pose.orientation` | body 到 world 的旋转 | 同左 |
| `twist.twist.linear` | IMU 原点的速度，在 body 中表达 | 同左 |

FAST-LIO 内部的 `state_point.vel` 和 `pub_x.vel` 都是 world 系速度。ROS `nav_msgs/Odometry` 规定 pose 在 `header.frame_id`，twist 在 `child_frame_id`，因此发布时统一用 `v_body = R_world_body.transpose() * v_world`。

之前 `/Odometry` 直接输出 world 速度，而 `/imu_propagate` 已做 body 转换，两者的 child frame 都为空。现在两个话题遵循同一约定，但因采样时间、LiDAR 校正和 IMU 外推不同，数值不保证逐帧相等。本次没有修改 IMU 外推积分及状态锚点逻辑。

`body` 是 FAST-LIO 使用的 IMU 坐标系；当前 `mid360.yaml` 使用 `/livox/imu`，所以这里指雷达内置 IMU。`world` 是 FAST-LIO 的局部参考系，不能仅凭名字认为它就是地理 ENU 或 PX4 local frame。

## 下游处理

- 实机 EGO Planner 默认订阅 `/imu_propagate`，轨迹生成要求 world 系速度。`ego_planner_run_in_exp.launch` 现在启用 `fsm/odom_velocity_in_body=true`，在里程计回调中使用完整姿态 `v_world = R_world_body * v_body`，包括横滚和俯仰。
- 仿真里程计仍直接提供 world 系速度。该参数默认 false，仿真 launch 不变。若自行更换里程计来源，应同步核对这个参数；frame 名称不会自动触发转换。
- GridMap 使用里程计的位姿，不使用其线速度，无需旋转位姿或修改点云坐标。
- 当前 `lio_to_mavros` 只用速度的模长做保护，旋转不改变速度大小，当前视觉位姿转发行为不变。
- 当前 px4ctrl 订阅 `/mavros/local_position/odom`，其现有速度控制代码按 body 速度使用反馈；本次没有更改该订阅或控制器。FAST-LIO 的 body 与飞控/base_link 仍需要安装外参才能对齐，不能直接替换话题并假定坐标相同。

## Ubuntu 编译和地面检查

同步代码后，在未解锁的地面状态停止服务、编译并重新启动：

```bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
catkin_make -j$(nproc)
source devel/setup.bash
sudo systemctl start grampusdrone.service
rostopic echo -n 1 /Odometry/child_frame_id
rostopic echo -n 1 /imu_propagate/child_frame_id
```

两个 child frame 都应为 `body`。启用了实机规划器时，还可检查：

```bash
rosparam get /drone_0_ego_planner_node/fsm/odom_velocity_in_body
```

结果应为 `true`。本次更新必须同时编译 FAST-LIO 和 EGO Planner；不要只替换 launch 而保留旧 FAST-LIO 二进制。

静止时速度近零无法验证坐标方向。地面缓慢平移时，分别用各消息自身的四元数把 body 速度转到 world，再与 world 位置的时间差分比较。偏航为 +90° 时，body 的 +X 速度应对应 world 的 +Y；反过来，world 的 +X 速度应对应 body 的 -Y。带横滚/俯仰时也要使用完整旋转。避免直接相减两个不同时间戳话题的原始速度分量。

## 未来改用 `/mavros/odometry/out`

现已增加独立的第一阶段输出 `/fast_lio/odometry_full`，含同帧角速度和变换后的协方差；定义、限制、参数和 Ubuntu 测试步骤见 [FAST-LIO 完整里程计](fastlio_full_odometry.md)。下文关于缺失字段和旧协方差的说明仍适用于原 `/Odometry`、`/imu_propagate`，不适用于新话题。新话题暂未接入 MAVROS。

速度坐标统一和第一阶段完整输出仍不足以直接把 FAST-LIO 话题 remap 到 MAVROS：

1. 确认本机 MAVROS odometry 插件实际订阅名、frame 参数和 TF 要求；不同安装配置可能不同。
2. 将 FAST-LIO 的局部参考系与输出参考系、IMU body 与飞行器 base_link 对齐。涉及安装平移时，速度还需要考虑角速度叉乘杆臂项；不能只改 `frame_id` 或 `child_frame_id` 的字符串。
3. 位姿、线速度、角速度、协方差和测量时间戳要使用一致的变换。当前两个源话题的角速度未填充，twist covariance 也未提供有效估计；`/imu_propagate` 没有完整传播协方差，`/Odometry` 原有 pose covariance 还存在发布后才赋值的问题，不能把这些默认零值当作完美测量。
4. 现有 PoseStamped 转发对位置减初始位置、对姿态右乘初始姿态逆，并重写为当前时间。这不是一套可以直接复制到 Odometry 的完整坐标变换。切换时需统一参考姿态、位置、速度、协方差及测量时间戳，再由 MAVROS 按配置处理 ENU/NED、FLU/FRD，避免重复变换。

速度坐标统一不代表定位质量已经得到保证，也不处理时钟跳变或 IMU 外推时间对齐问题。
