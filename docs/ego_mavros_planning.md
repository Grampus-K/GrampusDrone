# EGO 使用 MAVROS 里程计：独立地面规划配置

新增启动入口：`roslaunch ego_planner ego_planner_run_with_mavros.launch`。

这是规划坐标迁移的第一步，不启动 px4ctrl、不解锁、不起飞，不修改 systemd。现有 `ego_planner_run_in_exp.launch`、`ego_launch.sh` 和 `onboard_stack.launch` 保持原样。

## 输入与输出

| 项目 | 新配置 |
| --- | --- |
| EGO 状态与 GridMap 里程计 | `/mavros/local_position/odom` |
| 障碍物点云 | `/cloud_registered_mavros` |
| 规划参考系 | `planning_frame`，默认 `map`，必须等于实际 MAVROS odom 和转换点云的 header.frame_id |
| 速度处理 | `odom_velocity_in_body=true`，完整姿态将 base_link 速度旋转到规划参考系 |
| 目标 | `/ego_mavros/goal`，`geometry_msgs/PoseStamped`，检查 frame_id |
| 轨迹采样指令（仅查看） | `/ego_mavros/position_cmd`，不发布到控制器现用的 `/position_cmd` |
| 规划器/轨迹服务器节点名 | 与旧配置相同；必须二选一运行 |

新的 RViz 入口为 `mavros_rviz.launch`，或启动规划时带 `rviz:=true`。地图、轨迹 Marker、PositionCommand 都使用同一个 `planning_frame`。帧名参数只声明已有数值的坐标系，不执行 TF 转换。需要的点云转换仍由上一阶段的转换节点完成。

为保留旧行为，少量共享 C++ 代码支持可选 frame 参数：轨迹指令与 Marker 默认仍为 `world`；旧规划器默认仍接收 GoalSet。新配置启用带 frame 的 PoseStamped 目标，以及里程计/点云帧名检查。RViz 的 3D Nav Goal 同时产生的旧 GoalSet 被隔离到未使用的话题，避免发给旧规划器。

## Ubuntu 同步与编译

停止之前手动运行的规划/点云转换终端，确保不解锁，先检查工作区改动：

```bash
cd ~/GrampusDrone
sudo systemctl stop grampusdrone.service
git status
git pull --rebase origin master
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
catkin_make -j$(nproc)
source devel/setup.bash
```

这次有 C++ 修改，必须重新编译，不能只更新 launch。若本地有未提交源码修改，先保留处理，不要直接 reset --hard。

## 地面启动顺序

### 1. 只启动原有定位链路

若 systemd 已确认控制器、规划器都关闭，可继续使用它。否则保持服务停止，在终端 A 手动启动：

```bash
source ~/GrampusDrone/devel/setup.bash
roslaunch px4ctrl onboard_stack.launch start_controller:=false start_planner:=false
```

不要同时启动服务和手动定位 launch，也不要同时运行旧、新两套 EGO。

### 2. 确认点云转换与实际参考系

如果上一阶段的转换节点已经单独启动并正常运行，保持它运行即可；否则终端 B：

```bash
source ~/GrampusDrone/devel/setup.bash
roslaunch lio_cloud_to_mavros registered_cloud_to_mavros.launch
```

需要自定义安装参数时，使用上一阶段已经测试过的 `config:=...` 文件。检查：

```bash
rostopic echo -n 1 /registered_cloud_to_mavros/healthy
rostopic echo -n 1 /cloud_registered_mavros/header
rostopic echo -n 1 /mavros/local_position/odom/header
```

要求 healthy=true，两个 header.frame_id 相同。下例假定都是 `map`，如果实际是其他名称，必须使用实际名称。

### 3. 启动新 EGO 与 RViz

终端 C：

```bash
source ~/GrampusDrone/devel/setup.bash
roslaunch ego_planner ego_planner_run_with_mavros.launch planning_frame:=map rviz:=true
```

默认不再次启动点云转换器。若没有单独运行终端 B，可改为：

```bash
roslaunch ego_planner ego_planner_run_with_mavros.launch \
  planning_frame:=map rviz:=true start_cloud_converter:=true
```

可选 `cloud_config:=/home/orin/grampusdrone-config/registered_cloud_to_mavros.yaml` 指向之前测试的配置。只能运行一个点云转换器。

无显示器时 `rviz:=false`（默认）；需要单独打开专用 RViz：

```bash
roslaunch ego_planner mavros_rviz.launch planning_frame:=map
```

### 4. 核对订阅和规划参数

```bash
rosnode info /drone_0_ego_planner_node
rosparam get /drone_0_ego_planner_node/fsm/odom_velocity_in_body
rosparam get /drone_0_ego_planner_node/grid_map/frame_id
rosparam get /drone_0_ego_planner_node/visualization/frame_id
rosparam get /drone_0_traj_server/traj_server/frame_id
```

前者订阅必须包括 MAVROS odom、转换点云和 `/ego_mavros/goal`；速度参数为 true，三个 frame 参数与实际数据一致。新配置屏蔽相机深度输入，按转换后的点云建图。

默认参数集中在新 launch 顶部，可命令行覆盖：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `virtual_ground` | -0.1 m | 规划坐标系中的虚拟下边界，同时是目标最低 z |
| `virtual_ceil` | 3.0 m | 规划坐标系中的虚拟上边界 |
| `visualization_truncate_height` | 3.0 m | 地图显示截断高度 |
| `max_vel` | 0.5 m/s | 规划最大速度 |
| `max_acc` | 1.0 m/s² | 规划最大加速度 |
| `max_jer` | 5.0 m/s³ | 规划最大 jerk |
| `planning_horizon` | 7.5 m | 规划距离 |

**高度均为 PX4 本地坐标中的绝对 z，不是离地高度，也不是相对当前位置。** 默认数值只供初次地面查看，必须根据实际地面/障碍物位置检查 `virtual_ground < virtual_ceil`。比如地面 z=-0.3 m，不能仍把 z=0 当作地面。改变坐标或高度参数后重启规划器重新建图。

### 5. 查看地图并发一个地面测试目标

在 RViz 检查 Fixed Frame 与 `planning_frame` 一致，确认飞机、转换点云和膨胀障碍物位置一致，再使用工具栏的 **3D Nav Goal** 选择处于自由空间内的目标和高度。不要使用旧 RViz 配置。目标坐标错误会被拒绝，而不会默默改标签；目标低于 `virtual_ground` 也会被拒绝。目标姿态不作为终点 yaw 约束，轨迹沿用现有的 yaw 生成逻辑。

也可以手工发布 PoseStamped。以下只是格式示例，`x=1, y=0, z=1` 必须换成实际空闲位置：

```bash
rostopic pub -1 /ego_mavros/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

查看轨迹指令（有规划结果后才有输出）：

```bash
rostopic echo -n 1 /ego_mavros/position_cmd
rostopic info /ego_mavros/position_cmd
```

指令 header.frame_id 应等于实际规划参考系，目标、轨迹应避开真实障碍物。该话题不应有 px4ctrl 订阅；本阶段不要 remap 到 `/position_cmd`。

拆桨地面缓慢移动和转动机体，确认墙面、轨迹和 MAVROS 位置持续对齐。RViz 显示正常不等于可以飞行；它不能证明延迟、PX4 融合质量或控制器坐标处理已全部正确。

## 本阶段边界与回退

本次只完成第一步配置迁移和坐标语义衔接。尚未新增规划运行时的里程计/点云超时联动、PX4 重置后的地图清理、控制降级，也没有修改 px4ctrl 的速度处理。即使转换节点停止发布，EGO 仍可能保留旧地图/轨迹，因此不要接入自动控制。出现转换 FAULT、明显跳变或地图错位时，停止测试，排查并重启转换器和规划器清空旧状态。

回退只需 Ctrl+C 停止新 EGO 和新 RViz，再启动原来的：

```bash
roslaunch ego_planner ego_planner_run_in_exp.launch
# 另一个终端按需要打开原来的 RViz：
roslaunch ego_planner real_rviz.launch
```

使用旧配置时需要原有 FAST-LIO `/imu_propagate`、`/cloud_registered`。若点云转换由新 launch 启动，也会随 Ctrl+C 停止；若单独启动，可自行停止。无需修改 systemd 或回退 Git 提交。
