# FAST-LIO 完整里程计：第一阶段

本阶段新增 `/fast_lio/odometry_full`（`nav_msgs/Odometry`），用于地面对照和后续外部里程计适配。MID360 配置默认开启；其他配置未设置 `odometry_full/enabled` 时默认关闭。

现有 `/Odometry`、`/imu_propagate`、`lio_to_mavros`、`/mavros/vision_pose/pose`、EGO 和 PX4 参数不变。新话题没有接入 MAVROS，也没有直接改为 `/mavros/odometry/out`。它只在原有激光更新之后发布；不为高频 `/imu_propagate` 套用过期的激光协方差。

## 消息含义

| 字段 | 含义 |
| --- | --- |
| `header.stamp` | 当前激光帧结束时刻，与同帧 `/Odometry` 相同；不改成发送时刻 |
| `header.frame_id` | `world`，FAST-LIO 自己的局部参考系，尚未对齐 PX4 local/ENU |
| `child_frame_id` | `body`，MID360 内部 IMU 的坐标轴和原点，尚不是飞行器 `base_link` |
| `pose.pose` | IMU 原点在 `world` 的位置和姿态 |
| `twist.twist.linear` | `Rᵀ × v_world`，在 IMU `body` 中表达的线速度，单位 m/s |
| `twist.twist.angular` | 对齐帧结束时刻的陀螺仪读数减去激光更新后的陀螺偏置，IMU `body` 系，单位 rad/s |
| `pose.covariance` | 行优先 6×6，顺序 `[x,y,z,绕世界固定X/Y/Z轴的小角度]`；对角单位 m²、rad² |
| `twist.covariance` | 行优先 6×6，顺序 `[vx,vy,vz,wx,wy,wz]`，均为 IMU `body`；对角单位 (m/s)²、(rad/s)² |

这里不应用雷达中心高于 `base_link` 7 cm 的安装参数，也不把内部 IMU 原点当作雷达中心。下一阶段需要同时考虑 LiDAR↔IMU 外参和 LiDAR↔机体安装外参，转换位置、姿态、杆臂速度及协方差。

## 协方差和时间对齐

采用**同一激光更新后的状态与 `kf.get_P()`**，在全部字段填完后发布新消息。旧 `/Odometry` 的 pose covariance 存在发布后才赋值、位置/旋转状态块对应错误的问题，因此不能用旧协方差作为新话题的验证基准。为保留旧链路，这次不修改旧消息；未来使用协方差应使用新话题。

本工程误差状态顺序为 `pos(0..2), rot(3..5), 外参(6..11), vel(12..14), bg(15..17), ba(18..20), grav(21..22)`。MTK 姿态误差使用右扰动 `R_true = R Exp(δθ_body)`，不是世界系欧拉角误差。输出雅可比为：

```text
δp_world     = δp
δθ_world     = R δθ_body
δv_body      = [v_body]× δθ_body + Rᵀ δv_world
δω_body      = -δbg + δgyro

Cpose        = Jpose P Jposeᵀ
Ctwist_base  = Jtwist P Jtwistᵀ + diag_block(0, Cgyro)
```

保留 P 中的交叉相关项。角速度观测与滤波后状态并非独立，因为陀螺仪也用于滤波传播；当前没有对应交叉协方差。输出使用 `2 × Ctwist_base` 作为一阶模型下未知相关性的保守上界，而不是宣称完全精确的后验协方差。之后再对两个输出矩阵应用 `covariance_scale` 和各项标准差下限。不会修改滤波器 P 或 `mapping/*_cov`。

从当前帧最后一个 IMU 样本和队列中下一个样本插值到帧结束时刻，使用与滤波器相同的修正后时间戳；恰好命中样本时直接使用。每侧时间间隔均须不超过 `max_gyro_gap`，不外推、不用 ROS 当前时间代替。插值噪声采用线性加权协方差，避免假定相邻样本噪声独立。ROS IMU 协方差全零时使用配置噪声下限，`-1` 表示没有角速度测量时跳过。

陀螺仪插值仍有采样和运动模型近似。激光/IMU 时间偏移必须正确；有限时间间隔并不证明传感器已同步。ROS 时间以纳秒表示，输出一致性检查允许最多 1 µs 的浮点表示误差。

新话题遇到缺少插值两端 IMU、超时间间隔、无有效激光匹配、非有限状态或明显无效协方差时会跳过当前帧，并输出限频告警。只修复协方差的微小数值非对称/负特征值，明显错误不会通过简单取绝对值掩盖。

**协方差不是定位健康证明。** 错误初始化、运动退化、时钟跳变、地图漂移和模型误差不一定反映在 P 中。当前增加的是消息完整性检查，不是新的发散检测和自动重启逻辑；已有视觉链路健康判断保持原样。标准差下限仅是起始配置，必须用静止、平移、转动的数据检查，不能为了让 EKF2 接受而随意调小。

## 参数位置与关闭方法

参数在 `src/realflight_modules/FAST_LIO/config/mid360.yaml` 的 `odometry_full` 中，启动时读取：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enabled` | `true` | MID360 新话题开关 |
| `covariance_scale` | `1.0` | 输出协方差倍率，必须 ≥1 |
| `position_stddev_floor` | `0.05` | 位置标准差下限，m |
| `orientation_stddev_floor` | `0.03` | 姿态小角度标准差下限，rad |
| `linear_stddev_floor` | `0.10` | 线速度标准差下限，m/s |
| `angular_stddev_floor` | `0.03` | 角速度标准差下限，rad/s |
| `gyro_noise_stddev` | `0.02` | 每个陀螺仪样本噪声标准差的回退值/下限，rad/s；与滤波过程噪声 `mapping/gyr_cov` 不同 |
| `max_gyro_gap` | `0.02` | 插值每侧最大间隔，秒；200 Hz IMU 正常间隔约 0.005 s |

标准差会平方后放进协方差，不能把方差直接填到这些参数里。所有下限和时间间隔须为有限正数；无效配置只会关闭新输出并报错，不改变旧输出。

回退只需把 YAML 中 `enabled` 改成 `false`，然后重启服务，不需要撤销代码或改 PX4 参数。不要仅运行 `rosparam set`：节点不会动态读取该开关，而且重新 launch 会重新加载 YAML。

## Ubuntu 编译与地面测试

先确认 Ubuntu 已同步到包含这些新增文件的版本。停止服务后编译，避免旧二进制运行着造成误判：

```bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
catkin_make -j2
source devel/setup.bash

# 编译成功后，运行只依赖 Eigen 的数学测试
./devel/lib/fast_lio/fast_lio_odometry_covariance_test

# 机体静止放好再启动，不启用控制器/规划器，不解锁
sudo systemctl start grampusdrone.service
rosparam get /odometry_full
rostopic info /fast_lio/odometry_full
rostopic hz --wall-time /fast_lio/odometry_full
rostopic echo -n 1 /fast_lio/odometry_full
```

编译失败时先保留日志，不继续启动新版本。数学测试需要 `CATKIN_ENABLE_TESTING` 开启（catkin 通常默认开启）；若此前关闭，使用 `catkin_make -j2 -DCATKIN_ENABLE_TESTING=ON`。不需要额外安装 gtest。

新话题频率应接近 `/Odometry` 的激光更新频率，可能因无效帧跳过而降低，不是 200 Hz IMU 频率。无消息/频繁缺帧时查看：

```bash
journalctl -u grampusdrone.service -n 100 --no-pager
rostopic hz --wall-time /livox/imu
rostopic hz --wall-time /Odometry
```

不要直接放大时间间隔掩盖丢包或时间错误。随后运行自动对照脚本（需要 `rospy`、`nav_msgs` 和 `numpy`，通常随现有环境已有）：

```bash
python3 shfiles/check_fastlio_full_odometry.py --samples 100 --timeout 60
```

脚本按相同时间戳配对新旧消息，检查 frame、位姿、body 线速度、四元数归一化、时间单调性，以及两套协方差的有限值、对称性和半正定性；最后打印速度范围及最小标准差。`PASS` 只表示消息一致性通过。静止时另外观察位置和速度是否漂移，缓慢平移/转动时确认速度方向和角速度符号合理。旧 `/Odometry` 未填写角速度，因此不比较其角速度/协方差。

记录静止和缓慢运动数据，留给后续统计调参：

```bash
rosbag record -O fastlio_full_ground.bag /livox/imu /Odometry /fast_lio/odometry_full /mavros/vision_pose/pose
```

## 下一阶段边界

第一阶段不等于可以直接 remap 给 PX4。下一阶段仍需完成 IMU→`base_link` 安装旋转/平移及杆臂速度修正、world→输出参考系的完整变换、协方差传播、数据健康门控和重启后参考系/重置处理。之后再核对本机 MAVROS odometry 插件的 TF/坐标参数、PX4 1.17 外部视觉噪声和速度融合设置，并完成地面及飞行验证。

PX4 不应同时收到同一来源的两条独立外部视觉输入；当前阶段继续使用原 `/mavros/vision_pose/pose`。没有改 EKF2 的融合开关，新增话题不会改变 `/mavros/local_position/odom` 的消息坐标定义。
