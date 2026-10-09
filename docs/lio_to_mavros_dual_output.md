# lio_to_mavros 双模式：实施结果与 Ubuntu 地面测试

本次沿用一个 `lio_to_mavros_node`，默认仍是已经测试过的 `vision_pose`。
新增的完整里程计尚需 Ubuntu ROS 编译、集成测试和实机融合验证，不能据此直接飞行。

## 1. 模式与文件结构

| 启动参数 | 输入 | 正式输出 | 隔离预览 |
|---|---|---|---|
| 默认 `output_mode:=vision_pose` | `/Odometry` | `/mavros/vision_pose/pose` | 无 |
| `output_mode:=vision_pose odometry_preview:=true` | 旧输入 + `/fast_lio/odometry_full` | 原 pose 链路 | `/lio_to_mavros/odometry_preview` |
| `output_mode:=odometry` | `/fast_lio/odometry_full` | `/mavros/odometry/out` | 无 |

同时启用正式 odometry 和 preview 会启动失败。模式只在启动时读取，不支持飞行中切换。
预览有自己的 `/lio_to_mavros/preview/healthy` 和 `/lio_to_mavros/preview/status`，不参与旧 pose 就绪判断。
正式 odometry 使用 `/lio_to_mavros/healthy` 和 `/lio_to_mavros/status`。
模式、输出与输入话题冲突以及预览 remap 到 MAVROS 命名空间会报错。
节点无法禁止其他进程向飞控发送数据，接入前仍要检查 ROS 图。

包内分工：

- `src/lio_to_mavros.cpp`：模式选择和单线程循环。
- `include/lio_to_mavros/legacy_pose_gate.hpp`：原有位姿门控；原初始化、30 Hz 发布、发送时刻时间戳、阈值及重启规则保留。
- `odometry_transform.hpp`：无 ROS 依赖的刚体与协方差变换。
- `odometry_guard.hpp`：无 ROS 依赖的完整里程计时钟、稳定窗口与锁止状态。
- `full_odometry_bridge.hpp`：ROS 参数、消息、MAVROS 状态与 TF 校验。
- `config/full_odometry.yaml`：只供完整模式/预览使用的配置。
- `test/odometry_transform_test.cpp`、`test/bridge.test`：catkin 单元与 ROS 回归测试。

未改 FAST-LIO、EGO、px4ctrl 算法和点云节点输入。现有点云转换包不随本 launch 启动；
如果另行运行，它仍需独立验证自己的参考系假设，不能仅由新桥接器的 healthy 推断可用。

桥接器不再由 roslaunch 自动 respawn。进程退出后上层 launch 保持运行，桥接器保持停止，
避免重启时丢失旧参考原点；默认 pose 的正常输出行为未改变。需在地面人工检查、重启。
systemd 的已有整栈重启策略与时间监控策略不属于无缝恢复保证。

## 2. 几何定义与协方差

`T_X_Y` 将 Y 坐标转换到 X。W=FAST-LIO world，I=内部 IMU/body，L=LiDAR，B=base_link，A=输出参考系。
FAST-LIO 源码定义 `p_I = R_I_L p_L + t_I_L`。桥接器每秒检查运行中的 `/mapping` 外参，
只接受 `extrinsic_est_en: false`；外参变化/消失会锁止。输入必须为 `world/body`。

| 几何 | 值或来源 | 处理者 |
|---|---|---|
| `T_B_L` | 平移 `[0,0,0.07] m`，无安装旋转 | `full/lidar_in_base` |
| `T_I_L` | 运行时 `/mapping/extrinsic_R`、`extrinsic_T` | FAST-LIO 参数；不重复硬编码 |
| `T_I_B` | `T_I_L inverse(T_B_L)` | 桥接器 |
| 飞控 IMU 相对 B | `[0.04,0,0] m`，无旋转 | PX4 参数核对；桥接器不使用这段偏移 |

这里假设用户给出的雷达安装参考原点正好是厂家 LiDAR 坐标原点，与现有点云配置一致。
若实际指外壳中心，先更正安装变换。MID360 当前配置给出：

```text
R_I_B = Identity
r_I = t_I_B = [-0.011, -0.02329, -0.02588] m
p_W_B = p_W_I + R_W_I r_I
R_W_B = R_W_I R_I_B
v_B = R_I_B^T (v_I + omega_I × r_I)
omega_B = R_I_B^T omega_I
p_A_B = R_A_W p_W_B + t_A_W
R_A_B = R_A_W R_W_B
```

twist 始终表达在 B，不能再乘 world 对齐旋转。
例如固定 I 原点、绕 I 的 z 轴以 `+2 rad/s` 转动时，补偿的 B 线速度为
`[+0.04658,-0.022,0] m/s`；若实际绕固定 B 转动，输入 I 速度应抵消这一项。

上游完整里程计 pose 误差是 `[dp_W,dtheta_W]`，姿态为 world 固定轴小角度（左扰动），
不是 IMU 局部右扰动。雅可比为：

```text
J_pose = [ R_A_W  -R_A_W [R_W_I r_I]x ]
         [   0             R_A_W     ]
J_twist = [ R_B_I  -R_B_I [r_I]x ]
          [   0        R_B_I    ]
C_out = J C_in J^T
```

保留两个 6×6 块中的交叉项。拒绝非有限、明显不对称或非半正定协方差；只修复浮点量级误差
（对称误差阈值 `1e-7*max(1,maxabs(C))`，最小特征值容差 `-1e-9*max(1,maxabs(C))`）。
上游已有正方差下限；桥接器拒绝非正对角，不额外提高融合权重。
不包含 nav_msgs 无法携带的 pose–twist 跨块相关、安装外参不确定性和启动参考的不确定性。
初始参考视为冻结的坐标选择，不能宣称得到全部联合不确定性。

## 3. 参考系、MAVROS 与飞控 IMU

默认 `zero_position: true`，静止窗口通过后把当时 B 原点置零；`zero_yaw: false`，
保留 FAST-LIO world 的水平轴方向。`reference_yaw` 是显式固定 W→A yaw（弧度）。
设 `zero_yaw: true` 时，只消除初始 B yaw，再加 `reference_yaw`，保留 roll/pitch 和重力方向。
不会订阅 PX4 local odom 来持续回校，也不声明 FAST-LIO 初始 yaw 就是地理东/北。

正式输出 frame 为 `odom/base_link`；这表示外部定位本地参考，不保证原点与 PX4 local 相等。
预览强制为 `lio_preview_odom/lio_preview_base_link`，不广播 TF，不污染现有树。

核对依据：[MAVROS 1.20.1 odom.cpp](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp)。
该插件订阅 `odometry/out`、发布 `odometry/in`，发送 `LOCAL_FRD/BODY_FRD` 和 VISION estimator type。
插件只使用 TF 的旋转进行变换，不能依赖它处理原点平移。

| 工作 | 责任 |
|---|---|
| 内部 IMU → B、杆臂与协方差 | 本桥接器 |
| W → 冻结 A 参考、启动平移/yaw | 本桥接器 |
| ROS ENU 轴约定 → MAVLink 本地 FRD、FLU → FRD | MAVROS，通过 TF；桥接器不重复转换 |
| 外部参考与融合参考的估计器对齐 | PX4；GPS+EV yaw 的绝对航向含义需实机核对 |
| B → 飞控 IMU 杆臂 | PX4 的 EV_POS 和 IMU_POS 参数 |

正式输出前检查 TF：`odom_ned <- odom` 为 `[[0,1,0],[1,0,0],[0,0,-1]]`，
`base_link_frd <- base_link` 为 `diag(1,-1,-1)`，平移为零。
支持 MAVROS `/mavros/odometry/fcu/odom_parent_id_des` 与 `odom_child_id_des` 的重命名；
非该轴转换会锁止，缺 TF 初始等待、运行中丢失锁止。节点不自行广播这些 TF。

MAVROS 此版本从 ROS 原始时间戳产生 MAVLink 微秒时间，没有从 nav_msgs 取得 reset_counter/quality。
本实现不伪造这些字段；其默认值和你本机插件行为必须核对，尤其 `EKF2_EV_QMIN` 不可凭空要求质量分数。

PX4 1.17 依据：
[module.yaml](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/module.yaml)、
[EV 参数](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/params_external_vision.yaml)、
[位置补偿](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp)、
[速度补偿](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF/aid_sources/external_vision/ev_vel.h)。
源码使用 `ev_pos_body - imu_pos_body`，速度偏移使用角速度与该向量的叉积。
参数原点为飞行器重心。**只有 B 与 PX4 的重心参考重合**、轴向已核对、固件定义一致时，
本次输出 B 的几何对应 `EV_POS=[0,0,0]`、`IMU_POS=[0.04,0,0]`；
不是 `EV_POS_X=-0.04`。若 B 不在重心，两者都要换算至同一个重心参考，FRD 的 y/z 与 FLU 符号相反。
飞控板中心不一定是有效 IMU 芯片原点。旧 pose 输出仍是旧原点定义，回退时恢复原参数记录。
这里不提供自动写飞控参数的脚本，也没有“配置了但不生效”的飞控 IMU 参数。

## 4. 故障与频率边界

完整里程计逐扫描发布，保持 `header.stamp`，约 10 Hz；不会伪造 30 Hz。
相同时间戳且相同观测丢弃，不刷新到达时间；同时间戳不同值锁止。
先检查单调时钟超时，再处理排队消息，避免恢复到达的数据掩盖断流。
静止计时使用单调时钟；测量年龄使用 ROS 时间（需与上游处于同一时间域）。

初始化需连续静止 5 秒，检查线速度、角速度、位置漂移和姿态漂移。
正式模式还需新鲜、connected 的 MAVROS state 且未解锁；解锁后才启动会锁止。
已开放后允许正常解锁运行，不自动归零。
完整路径故障一律锁止到进程重启：NaN/Inf、四元数、协方差、位置/速度/角速度超限、
时间戳回退/过旧/未来、姿态/位置跳变、发布序号回退、时钟跳变、断流、正式模式断连。
seq 32 位溢出被区别处理；没有 reset 字段时，小幅或未显现的地图重置仍可能无法检测，
所以不得运行中主动重置 FAST-LIO，也不能将这些启发式门槛当成准确性证明。

默认最大年龄 0.5 s、未来容差 0.05 s、断流 0.5 s、时钟跳变容差 0.5 s。
阈值是地面验证起点，安装变化、真实频率和延迟需重新核对。
短于超时的缺帧保留原参考；初始化期间较长缺帧重新累计稳定窗口；开放后超时不自动恢复。
旧 pose 预放行自动重启 FAST-LIO 时，预览可能锁止，这不会阻断旧 pose；地面整栈重启后再测预览。

## 5. Ubuntu：同步、编译、自动化测试

全程拆桨、未解锁，先停止服务和其他手动启动实例。保留本地硬件配置，不使用 reset --hard。

```bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
git status --short
# 如有本地修改，先保存/提交并处理冲突，再继续。
git pull --ff-only origin master
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash
sudo apt-get install libeigen3-dev ros-noetic-rostest ros-noetic-tf2-ros python3-numpy
catkin_make -j2
catkin_make run_tests_lio_to_mavros -j2
catkin_test_results build/test_results/lio_to_mavros
source devel/setup.bash
```

ROS 测试只使用合成消息、隔离 `/test/*` 话题和测试 TF，不要同时运行真实飞控链路。
它检查默认 pose 回归、预览隔离、正式模式不发 pose、扫描时间戳、重复帧、断连锁止、
已解锁启动拦截、旧输入超时以及非法模式/remap 拒绝。
数学测试覆盖安装组合/逆变换、7 cm、yaw/roll/pitch、安装旋转、杆臂符号、四元数 ± 等价、
两个雅可比有限差分、100 组随机 PSD/交叉协方差、非法输入和时间/重启状态。

本次 Windows 实际完成：16 项 C++ GoogleTest 全通过（Zig/Clang 0.13、Eigen 3.4、GoogleTest 1.14，
本机因编译器 SSE 内联汇编限制仅测试命令关闭 Eigen 向量化，仓库构建不关闭）。
Windows 无 ROS，未执行 ROS 节点编译、rostest、Orin 或 PX4 实机融合测试。

## 6. 默认旧模式回归

```bash
python3 shfiles/wait_for_stable_clock.py
roslaunch px4ctrl onboard_stack.launch start_controller:=false start_planner:=false
```

另开已 source 环境的终端：

```bash
rosparam get /lio_to_mavros/output_mode
rostopic echo -n 1 /lio_to_mavros/healthy
rostopic hz /mavros/vision_pose/pose
rosnode info /lio_to_mavros
rostopic info /mavros/odometry/out
bash shfiles/wait_for_stack.sh 120
```

应为 vision_pose、静止后 healthy=true、旧 pose 约 30 Hz，本节点不发布正式 odometry。
现有 planner/controller 默认不启动。手动 roslaunch 不会启动系统时间监控脚本，勿运行中调整系统时间。

## 7. 预览检查（Ubuntu 首次推荐）

停上一步 launch，再启动：

```bash
roslaunch px4ctrl onboard_stack.launch \
  output_mode:=vision_pose odometry_preview:=true \
  start_controller:=false start_planner:=false
```

另开终端：

```bash
rostopic echo -n 1 /lio_to_mavros/preview/status
rostopic hz /fast_lio/odometry_full
rostopic hz /lio_to_mavros/odometry_preview
rostopic echo -n 1 /lio_to_mavros/odometry_preview
rostopic info /lio_to_mavros/odometry_preview
rostopic info /mavros/odometry/out
python3 shfiles/check_lio_bridge_odometry.py --samples 100 --timeout 60
rostopic hz /mavros/vision_pose/pose
```

预览应约等于上游 10 Hz，frame 是 `lio_preview_odom/lio_preview_base_link`，MAVROS 不订阅预览。
检查脚本匹配同时间戳输入输出，检查有限性、对称/PSD、杆臂和速度/协方差传播、参考固定；
首对数据只推算固定参考，所以它不验证绝对航向、安装准确性，也不替代飞控融合验证。
分别保持静止、沿 x/y/z 平移、绕 x/y/z 缓慢旋转并停稳，重复运行检查；确认方向及速度回零。
同时看旧 pose 频率、healthy 和就绪检查。与 MAVROS local odom 比较前先明确固定参考对齐，
不能要求两个估计器逐点相等，也不能把融合输出反过来实时修正外部观测。

如要测试预览故障隔离，只在地面把运行时 `/mapping/extrinsic_est_en` 暂改为 true，预览应锁止而旧 pose 仍输出；
测试后恢复原值并重启整栈。该参数服务器操作用于触发桥接检查，不代表 FAST-LIO 动态改变了内部配置。
更推荐先通过隔离 rostest 完成此检查。

## 8. 正式 odometry 接入前核对

在 Orin 上保存版本与配置：

```bash
rosversion mavros
rosversion mavros_extras
rostopic info /mavros/odometry/out
rosparam get /mavros/odometry
rosparam get /mapping
rosrun tf2_ros tf2_echo odom_ned odom
rosrun tf2_ros tf2_echo base_link_frd base_link
```

后两条分别持续运行，读完 Ctrl+C。必须看到前文纯旋转，不能凭 frame 字符串判断。
缺少 TF 时检查安装版 MAVROS 的静态 TF 发布配置；先查现有树，不盲目重复启动发布器。
标准版本通常由 MAVROS 提供轴转换。本桥接器检查失败时不会发送正式数据。
`/mavros/odometry/out` 应有 MAVROS 订阅者，pose 与 odometry 不应同时有外部测量发布者。

在 QGroundControl 的 MAVLink Console 只读核对并记录：

```text
ver all
param show EKF2_EV_CTRL
param show EKF2_EV_POS_*
param show EKF2_IMU_POS_*
param show EKF2_EV_DELAY
param show EKF2_EV_QMIN
param show EKF2_EV_NOISE_MD
```

实际固件若不同于 v1.17.0，以本机源码/参数定义为准。按第 3 节核对原点、轴和 4 cm。
GPS+EV yaw 时必须明确绝对航向对齐，不能把 `zero_yaw` 当作校准地理航向。
保留原有已验证融合项，初次不要新增速度融合。若当前已开启速度融合，应先单独决定地面测试配置，
不要照抄一个掩码值。v1.17 的 EV_CTRL bit 2 对应速度，其余位需保留原记录。
约 10 Hz 是否能可靠融合必须看飞控结果；不能用反复更新时间戳来满足频率要求。

## 9. 正式地面接入与回退

停服务/旧 launch，在地面未解锁状态启动：

```bash
roslaunch px4ctrl onboard_stack.launch \
  output_mode:=odometry odometry_preview:=false \
  start_controller:=false start_planner:=false
```

另开终端：

```bash
rostopic echo -n 1 /lio_to_mavros/status
rostopic echo -n 1 /lio_to_mavros/healthy
rostopic info /mavros/vision_pose/pose
rostopic info /mavros/odometry/out
rostopic hz /mavros/odometry/out
python3 shfiles/check_lio_bridge_odometry.py --topic /mavros/odometry/out --samples 100
bash shfiles/wait_for_stack.sh 120
```

旧 pose 不应再有本桥接器发布者。ROS 有消息/订阅者只证明链路的一段，不证明 PX4 接收或融合。
QGC MAVLink Console 检查：

```text
listener vehicle_visual_odometry 5
listener estimator_status_flags 5
listener estimator_aid_src_ev_pos 5
listener estimator_aid_src_ev_hgt 5
listener estimator_aid_src_ev_yaw 5
listener estimator_aid_src_ev_vel 5
```

部分固件未编译相关 uORB 话题时用 `uorb top` 确认可用名称，再结合 ULog 查看。
检查 sample 时间、频率、position/velocity frame、方向、方差、reset_counter、fusion_enabled/fused、
创新/检验比和拒绝标志。同步时钟与 EV_DELAY 需结合实测延迟，而非盲填零。
保存接入前后参数与日志；位置姿态确认后再单独讨论开启速度融合，不一次改协议、噪声、偏移和融合开关。

回退无需回滚 Git。停新 launch，恢复原 PX4 参数记录，再启动：

```bash
roslaunch px4ctrl onboard_stack.launch \
  output_mode:=vision_pose odometry_preview:=false \
  start_controller:=false start_planner:=false
```

确认正式 odometry 发布停止、旧 pose 恢复、healthy 和就绪检查通过。

## 10. systemd 配置

手动测试通过后，使用专用 drop-in，不覆盖已有硬件配置：

```bash
sudo systemctl stop grampusdrone.service
sudo mkdir -p /etc/systemd/system/grampusdrone.service.d
sudo tee /etc/systemd/system/grampusdrone.service.d/lio-output.conf >/dev/null <<'EOF'
[Service]
Environment=LIO_OUTPUT_MODE=vision_pose
Environment=LIO_ODOMETRY_PREVIEW=true
Environment=START_CONTROLLER=false
Environment=START_PLANNER=false
EOF
sudo systemctl daemon-reload
sudo systemctl start grampusdrone.service
journalctl -u grampusdrone.service -b -n 100 --no-pager
```

这是预览配置。正式地面接入时，停服务，把同一文件改成 `LIO_OUTPUT_MODE=odometry`、
`LIO_ODOMETRY_PREVIEW=false`，保持 controller/planner=false，然后 daemon-reload/start。
回退同一文件为 `vision_pose` 和 `false`，并恢复此前 PX4 参数记录。
可用 `LIO_FULL_CONFIG=/home/orin/自定义路径.yaml` 指定独立配置，文件内容结构与包内 YAML 一致。
`systemctl cat grampusdrone.service` 检查多个 drop-in 的最终优先级。
