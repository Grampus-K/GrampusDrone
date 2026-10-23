# D455 双目 OpenVINS 接入（ROS Noetic / PX4 1.17）

本次已增加独立的 `openvins_bridge` 包、固定版本源码下载器、双目启动、出厂参数导出、Kalibr 导入及 PX4 地面桥接。原有 FAST-LIO、自启动、px4ctrl、规划器均未切换。当前 D455 只有出厂标定，因此首先运行预览；这一步能检查链路和轨迹，不能据此认定定位精度或可飞行。

```text
D455 左右红外 + 内置 IMU
  → camera_guard（关闭投射器、双目配对、时间与断流检查）
  → /vio_inputs/* → OpenVINS → /ov_msckf/odomimu
  → openvins_bridge → /openvins_bridge/odometry_preview
                   → /mavros/odometry/out（仅标定后的未解锁地面模式）
GNSS → PX4 EKF2 ← 外部视觉；px4ctrl 继续读取 /mavros/local_position/odom
```

## 当前继续入口（2026-10-23：采流已通过，开始标定准备）

**这台设备从本节继续，不重做第 1 节依赖安装或驱动文档的 SDK 重编。** 后续主线是本文第 4 节的标定，再到第 3 节预览，最后第 5 节 PX4 地面联调。

已完成：Orin 上双目约 30 Hz、组合 IMU 约 400 Hz，`/d455_input/ready=True`；Windows 同版本 Viewer 也能采样，用户观察到静止角速度接近零、加速度模长约 9.77 m/s²，转动时响应正常。这通过了基本功能检查，不是精度、长期连续性或同步验收。Windows 也复现 IMU 默认内参/CAD 外参提示，说明此提示不是 ROS 独有的问题，尚未排除共有 SDK 或设备/固件因素。

当前保留 SDK 2.56.3、ROS 驱动 2.3.2、固件 5.16.0.1 和现有采样设置。先不用第 2 节的 `export_factory_calibration.py`：当前 TF 可能来自 CAD 回退，现有导出脚本的“factory”来源描述不足以表达这一情况。缺少可读标定表不必成为离线标定的阻碍，但不能把默认值记录为本机实测出厂标定。详细驱动排查记录到此告一段落；若之后出现持续 Motion Module 通知、断流或异常数值，再保留对应日志排查。

### 现在先做：录一份短的六姿态 IMU 数据

这用于检查各轴偏置和尺度是否有明显异常，**不是完整内参标定，也不写 EEPROM**。先关闭 Windows Viewer，把相机接回 Orin；保持整机服务停止，在地面单独采集。

终端 A（沿用 `.bashrc` 中已经配置好的环境，不新增 source 行）：

```bash
sudo systemctl stop grampusdrone.service
source ~/GrampusDrone/devel/setup.bash
roslaunch openvins_bridge d455.launch 2>&1 | tee ~/d455_six_pose_camera.log
```

终端 B：先确认 `ready` 为 True，再录 4 分钟 CSV。录制期间该命令没有持续终端输出属于正常现象：

```bash
source ~/GrampusDrone/devel/setup.bash
timeout 5s rostopic echo -n 1 /d455_input/ready
mkdir -p ~/grampus_vio_data
D455_POSE_CSV="$HOME/grampus_vio_data/imu_six_pose_$(date +%Y%m%d_%H%M%S).csv"
timeout --signal=INT 240s rostopic echo -p /vio_inputs/imu > "$D455_POSE_CSV"
wc -l "$D455_POSE_CSV"
ls -lh "$D455_POSE_CSV"
```

录制开始后，用稳定支撑让相机依次处于六种大致相反的静止朝向，使重力分别接近 IMU 三个轴的正、负方向。每种朝向放稳约 20 秒，中间缓慢翻转，避免碰撞镜头、挤压接口或牵拉 USB 线。不要求初检时精确知道各轴对应哪个机壳面；无法安全放置时用支架，不强行立放。结束后提供该 CSV 和 `~/d455_six_pose_camera.log`，先检查静止段均值、模长及变化情况；样本行数只用于确认文件不是空的，不代替时间戳检查。

`timeout` 到时退出是预期行为，不代表采集失败。若 `ready=False`、CSV 为空或只有表头，先检查本次原始流和 launch 日志，不能用 Windows 数值检查代替本次 Orin 采集。

### 随后按这个顺序推进

1. 根据六姿态初检决定是否需要进一步 IMU 内参校正；粗略静置不能测全轴间误差，不直接生成或写入内参。
2. 本机用户目前没有标定板，先准备 Kalibr 兼容的 AprilGrid。可参考 [Kalibr 标定板说明](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)，选择适合近距离双目同时观察的尺寸，优先 A2 大小的平整硬质哑光板。购买时确认是 Kalibr 可用的 AprilGrid，保留原始 PDF/生成参数，包括 tag 家族、ID 排列、行列数、单个 tag 尺寸和间隔，不能只凭“AprilTag 标定板”商品名称判断兼容。自行打印须关闭适应页面缩放，贴到平板上并实测尺寸；target YAML 里的 `tagSize` 用米，`tagSpacing` 是间隔与 tag 边长之比，不是米。没有板也能先完成上述 IMU 初检；不要用手机屏幕临时代替实物做最终标定。
3. 在最终确定的 IMU 数据流上采集长时间静态数据并估计噪声，按第 4 节进行双目标定、相机/IMU 联合标定和结果导入。`--imu-models calibrated` 假设输入已校正，不会自动解决尺度/轴间误差；如需要校正数据流，还需先实现并验证运行时一致的校正方式。
4. 使用导入的配置执行第 3 节地面预览；通过后再进入第 5 节。当前先不启动 PX4 外部视觉桥接。

## 0. 从 Windows 同步到 Ubuntu

GitHub 仓库为 `https://github.com/Grampus-K/GrampusDrone.git`，本次接入代码位于 **master** 分支。在 Ubuntu 工程中先核对远端、分支和本地修改：

```bash
cd /home/orin/GrampusDrone
git status --short
git remote -v
git branch --show-current
```

如果 `origin` 指向上述 GitHub 仓库、当前为 `master`，且工作区干净，执行：

```bash
git pull --ff-only origin master
```

如果当前为 `main`、`origin` 指向 GitLab，或有本地改动，先保留本地工作并核对分支，不直接覆盖或强制重置。需要并行保留原工程时，可在新目录克隆 GitHub 的 master；之后编译/启动命令中的工程路径也要同步修改：

```bash
git clone --branch master --single-branch \
  https://github.com/Grampus-K/GrampusDrone.git ~/GrampusDrone-openvins
```

Git 只同步集成代码，不包含设备标定、构建产物和 OpenVINS 第三方源码；下一节的下载器会在 Ubuntu 准备固定版本源码。

### 可选：通过文件同步

本次另提供独立增量包 `GrampusDrone-openvins-sync-20261023.tar.gz`，适合 Ubuntu 无法访问 GitHub 的情况，包含本包、下载器和本文档。压缩包是制作时的快照，后续更新以 GitHub 为准。

在 Windows PowerShell 中，将 `192.168.1.100` 替换为机载机实际 IP，用户名按实际修改：

```powershell
scp "C:\Users\Ke\Desktop\GrampusDrone-openvins-sync-20261023.tar.gz" orin@192.168.1.100:/home/orin/
```

也可以通过 U 盘把同一文件复制到 Ubuntu 的 `/home/orin/`。然后在 Ubuntu 执行（项目不在此处时修改 `cd`）：

```bash
cd /home/orin/GrampusDrone
tar -tzf /home/orin/GrampusDrone-openvins-sync-20261023.tar.gz
tar --backup=numbered -xzf /home/orin/GrampusDrone-openvins-sync-20261023.tar.gz
chmod +x shfiles/fetch_openvins.py src/realflight_modules/openvins_bridge/scripts/*.py \
  src/realflight_modules/openvins_bridge/test/test_bridge_ros.py
```

解压应看到 `docs/openvins_d455.md`、`shfiles/fetch_openvins.py` 和 `src/realflight_modules/openvins_bridge/`。若存在同名文件，GNU tar 会保留编号备份；先核对本机改动。同步包不覆盖 `.gitignore`；如需防止本机第三方源码和设备标定被误提交，可将以下两行加入 Ubuntu 工程的 `.git/info/exclude`：

```text
/src/third_party/open_vins/
/src/realflight_modules/openvins_bridge/config/local/
```

接着执行下一节的依赖、源码下载和编译步骤。**不要复制 Windows 的 build/devel，也不要把 `openvins_bridge` 再套进一层同名文件夹。** 源码下载器会在 Ubuntu 本地准备 OpenVINS。

## 1. 在机载 Ubuntu 上准备依赖和编译

以下命令在 Ubuntu 20.04 / ROS Noetic 的机载机执行，工程路径按实际修改。此处的 Windows 开发机未提供 ROS，不能代替机载编译与硬件验证。

**Jetson 注意：如果 `pkg-config --modversion opencv4` 是 NVIDIA 的 4.5.4，且系统 cv_bridge 链接 OpenCV 4.2，请先执行 [Jetson OpenCV 依赖修复](openvins_jetson_opencv.md)，用其中命令替代本节。** 该方案保留 NVIDIA OpenCV、在独立 overlay 重编译 cv_bridge，并用 `-DENABLE_ARUCO_TAGS=OFF` 关闭本次不需要的标记跟踪功能。下面这套 apt 命令仅适用于 OpenCV 开发包和 cv_bridge 使用一致的 Ubuntu 系统库的环境。

```bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
python3 shfiles/fetch_openvins.py
sudo apt update
sudo apt install python3-numpy python3-yaml python3-nose python3-opencv \
  ros-noetic-rostest ros-noetic-dynamic-reconfigure ros-noetic-message-filters \
  ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-image-transport \
  libeigen3-dev libboost-all-dev libceres-dev libopencv-dev libopencv-contrib-dev
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make -j2 -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
rospack find ov_msckf
rospack find realsense2_camera
catkin_make run_tests_openvins_bridge -j2
catkin_test_results
```

下载器固定 OpenVINS 提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`（2.7.0），并验证 SHA256；下载目录不进入本工程 Git。新机器仍需执行下载器。已有目录不会被覆盖。

相机 launch 对照的是 **ROS1 realsense-ros 2.3.2** 的接口，需要事先安装相应 ROS1 驱动及匹配的 librealsense。不要安装 ROS2 分支充当本包依赖。Jetson 上优先保留已验证的 SDK/内核组合，通过 USB3 连接；没有驱动时参照 [该版本说明](https://github.com/realsenseai/realsense-ros/tree/2.3.2) 安装，再运行上述编译。驱动版本变更后重新核对 launch 参数和时间戳。

工程原来若已装 Ceres/OpenCV，先检查版本和 CMake 缓存，避免 apt 安装和 `/usr/local` 的另一套库混用。**OpenVINS 的 OpenCV 必须与 cv_bridge 一致**，尤其注意 Noetic 默认 OpenCV 4.2 与手动安装的 4.5。编译日志中的 OPENCV/CERES 路径不一致时先解决依赖，不通过忽略链接警告继续联调。

## 2. 启动相机，导出这一台 D455 的出厂参数

先确认 `rospack find realsense2_camera` 和 `rosversion realsense2_camera` 成功。若报 `Resource not found: realsense2_camera`，且未安装 SDK/驱动，先执行 [Jetson D455 驱动安装](d455_jetson_driver.md)，再回到本节；OpenVINS 编译和桥接测试通过不代表相机驱动已就绪。

Jetson 已按专用 OpenCV 说明构建 overlay 时，`.bashrc` 中的已有 source 行应依次为 ROS → livox_ws → grampus_cv_bridge_ws → GrampusDrone，主工程必须最后加载。不要在主工程后再次 source cv_bridge 工作空间。当前终端不确定时手动按此顺序加载；正常新终端使用 `.bashrc` 即可。其他环境每个新终端先 `source ~/GrampusDrone/devel/setup.bash`。

建议在地面停止已有整机服务进行独立测试。若提示 unit changed on disk，先执行 `sudo systemctl daemon-reload`，再 stop，并用 `systemctl status grampusdrone.service --no-pager -l` 确认 `inactive (dead)`：

```bash
sudo systemctl stop grampusdrone.service
roslaunch openvins_bridge d455.launch
```

另一终端检查：

```bash
rostopic hz /vio_inputs/left/image_raw
rostopic hz /vio_inputs/right/image_raw
rostopic hz /vio_inputs/imu
rostopic echo -n 1 /d455_input/ready
rosrun rqt_image_view rqt_image_view
```

若显示 no new messages 或 ready=False，先按 [驱动说明第 6 节](d455_jetson_driver.md#6-launch-启动但无图像imu) 检查 `/d455/infra1/image_rect_raw`、`/d455/infra2/image_rect_raw`、`/d455/imu` 原始流。rqt 中也先选原始图像话题。仅 launch 未退出不表示相机采流成功。固件 5.16.0.1 不可搭配旧 SDK 2.50.0，升级与重编步骤见同一文档；仍提示 IMU 工厂标定无法读取时，先不要导出参数。

左右图像均为 848×480、30 Hz；IMU 默认请求 gyro 400 Hz、accel 100 Hz，由 RealSense 按陀螺仪时刻插值组合。以本机 SDK 枚举支持档位和实际输出为准；可以用 `gyro_fps:=... accel_fps:=...` 调整（d455/preview/px4_ground 均支持），后续标定与运行保持一致。此前固定 250 Hz 在本机不支持，驱动会回退到 100 Hz；修改采样参数不能解决固件/SDK 的格式不兼容。图像应清楚、有纹理，左右不是同一张图。`camera_guard` 确认 `emitter_enabled=0` 后才转发，避免主动红外投射图案被当作场景特征。亮度/曝光需根据实际光照检查；下视时注意机体、起落架遮挡。

```bash
cd ~/GrampusDrone
mkdir -p src/realflight_modules/openvins_bridge/config/local
rosrun openvins_bridge export_factory_calibration.py \
  --upstream "$PWD/src/third_party/open_vins" \
  --output "$PWD/src/realflight_modules/openvins_bridge/config/local/d455_factory"
```

脚本从实时 CameraInfo 和图像 optical frame 的 TF 导出内参、畸变和工厂相机/IMU 空间外参，同时记录来源。相机时差暂设为 0，IMU 噪声暂用上游示例，均明确标为 **factory_preview**。这些不是实测联合标定。若驱动输出的畸变模型不支持、基线不对或分辨率不符，脚本会报错，不会编造参数。目录已存在也不会覆盖。

## 3. 运行 OpenVINS 预览

保持相机 launch 运行，另一终端执行：

```bash
cd ~/GrampusDrone
roslaunch openvins_bridge preview.launch start_camera:=false \
  config_dir:="$PWD/src/realflight_modules/openvins_bridge/config/local/d455_factory"
```

先静置数秒供初始化。静态初始化可能等待一次轻微手动运动触发，然后重新静置；观察 OpenVINS 日志确认初始化完成。桥接还要求约 3 秒静止稳定。整个操作保持未解锁，手持/推车验证。

```bash
rostopic echo /openvins_bridge/status
rostopic hz /openvins_bridge/odometry_preview
rostopic echo -n 1 /openvins_bridge/odometry_preview
```

RViz 固定坐标设 `vio_preview_odom`，添加对应 Odometry。没有机体安装外参时，child 是 `vio_preview_imu`，轨迹表示 IMU 原点；不能当作飞控机体原点。依次平移、升降、转动并回到原处，检查尺度、垂直方向、回程误差和静置漂移。预览不会发布 MAVROS 外部视觉。

桥接保留源时间戳，处理 IMU→机体杆臂位置/速度及协方差；发布上限 30 Hz。输入时间倒退、跳钟、初始化后的断流或异常跳变会锁存 FAULT，恢复输入也不自动续发，需地面检查后重启。默认阈值是初始检查参数，需通过实际数据评估。

建议记录可复现数据，先确认磁盘余量：

```bash
rosbag record -O d455_preview.bag /vio_inputs/left/image_raw \
  /vio_inputs/right/image_raw /vio_inputs/imu /d455_input/ready \
  /ov_msckf/odomimu /ov_msckf/poseimu /openvins_bridge/odometry_preview \
  /openvins_bridge/status /tf /tf_static
```

## 4. 完成联合标定并导入

使用与实际运行完全相同的 **左右红外、848×480、IMU 组合方式和坐标轴**。保留投射器关闭状态。标定板尺寸、AprilGrid 行列数/间距必须按实物填写；相机出厂参数不能替代时间偏移标定、IMU 噪声测量或机体安装外参。

1. 按 [Kalibr 相机标定](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) 准备 target YAML，采集覆盖视场、姿态和距离变化的标定板图像。用于标定的 bag 必须包含 `/vio_inputs/left/image_raw` 和 `/vio_inputs/right/image_raw`。
2. 测量静态 IMU Allan 噪声，生成含四个 noise/random_walk 参数、`update_rate` 和 `rostopic: /vio_inputs/imu` 的 `imu.yaml`。用于噪声测量和联合标定的传感器流应相同。先检查单位为 rad/s 和 m/s²。
3. 联合标定时让两个相机持续看见标定板，充分激励三个转轴及平移，避免运动模糊；检查重投影误差、IMU 残差、时间偏移及重复标定的一致性。命令示例（路径替换为实物和实测文件）：

```bash
kalibr_calibrate_cameras --bag stereo.bag \
  --topics /vio_inputs/left/image_raw /vio_inputs/right/image_raw \
  --models pinhole-radtan pinhole-radtan --target target.yaml
kalibr_calibrate_imu_camera --bag joint.bag \
  --cam stereo-camchain.yaml --imu imu.yaml --imu-models calibrated \
  --target target.yaml
```

以本机 Kalibr 的 `--help` 和 [联合标定说明](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration) 核对参数及生成文件名。这里使用 `calibrated` IMU 模型，导入器不自动转换 Kalibr 的尺度/轴间非正交标定结果。输出应包含 `cam0/cam1` 的 `T_cam_imu`、`timeshift_cam_imu`。如果只得到相机间外参，尚未完成相机/IMU 联合标定。

4. 按真实安装测量 `T_base_imu`，写入独立的 `mount.yaml`。它满足 **p_base = R_base_imu × p_imu + t_base_imu**，平移单位米。base 为 ROS FLU（前、左、上），IMU 为实际 `/vio_inputs/imu` 的 frame，必须计入下视安装旋转和 IMU 到机体原点的偏移。不要填单位阵占位，也不要直接拿 camera optical 的外参当 IMU 外参。

导入联合标定结果；不提供 `--mount` 也可继续预览，但不能输出飞控：

```bash
cd ~/GrampusDrone
rosrun openvins_bridge prepare_calibration.py \
  --upstream "$PWD/src/third_party/open_vins" \
  --camchain /absolute/path/joint-camchain-imucam.yaml \
  --imu /absolute/path/imu.yaml --mount /absolute/path/mount.yaml \
  --output "$PWD/src/realflight_modules/openvins_bridge/config/local/d455_calibrated"
```

修改分辨率、替换 D455、调整传感器之间的刚性安装后，重新核对标定；移动相机相对机体位置后重测机体外参。

## 5. PX4 1.17 未解锁地面联调

先用联合标定配置重复预览检查。停止预览和已有整机服务，确保没有旧 `lio_to_mavros`、vision_pose、vision_speed 发送者，且只运行一个 MAVROS。桥接会检查 ROS 图中的其他外部视觉发布者，冲突时停发；仍需检查飞控是否从另一链路收到外部视觉。

```bash
sudo systemctl stop grampusdrone.service
cd ~/GrampusDrone
roslaunch openvins_bridge px4_ground.launch \
  config_dir:="$PWD/src/realflight_modules/openvins_bridge/config/local/d455_calibrated" \
  fcu_url:=/dev/ttyACM0:57600
```

若已手动启动 MAVROS，加 `start_mavros:=false`；若相机仍独立运行，加 `start_camera:=false`。按真实设备修改串口/波特率。飞控须连接且未解锁，MAVROS 必须提供标准 `odom_ned←odom`（ENU→NED）与 `base_link_frd←base_link`（FLU→FRD）静态 TF。桥接输出 ROS ENU/FLU，交给 MAVROS 转换，不能在别处再交换轴。

```bash
rostopic info /mavros/odometry/out
rostopic echo /openvins_bridge/status
rostopic hz /mavros/odometry/out
rostopic echo -n 1 /mavros/local_position/odom
```

PX4 MAVLink Console 先记录实际固件和当前参数，再观察接收数据：

```text
ver all
param show EKF2_EV_CTRL
param show EKF2_EV_DELAY
param show EKF2_EV_NOISE_MD
param show EKF2_EV_QMIN
param show EKF2_GPS_CTRL
param show EKF2_HGT_REF
listener vehicle_visual_odometry
uorb top
```

**接收外部里程计不等于融合成功。** 在 ULog/对应 aid source 中确认接受率、innovation、拒绝/超时及高度/航向行为。PX4 1.17 的 `EKF2_EV_CTRL` 位 0/1/2/3 分别是水平位置/垂直位置/三维速度/航向，不沿用旧固件 `EKF2_AID_MASK`。本次没有写入任何 PX4 参数：是否融合视觉高度/航向，要根据真实外参与航向对齐、现有 GNSS/磁力计/气压计/测距配置决定；不提供统一位掩码让你盲填。

OpenVINS 的起始水平航向是任意的。`reference_yaw` 只支持明确、固定的地面初始对齐，不允许持续跟随 PX4 输出对齐形成反馈。如果融合视觉航向，必须先验证与 PX4 航向的一致性及 GNSS 场景转换；不融合视觉航向也需验证该固件的外部视觉坐标对齐行为。`EKF2_EV_DELAY` 根据实际测量调整（毫秒），其设定需按固件要求重启；不能用“收到时刻”替换采样时间去掩盖延迟。协方差是否采用消息，取决于 `EKF2_EV_NOISE_MD`。

`px4_ground.launch` **检测解锁立即锁存停发**，不用于起飞。先拆桨完成方向、尺度、延迟、杆臂和断流试验。此限制来自当前实现尚缺飞行所需的健康判断和恢复语义；不是改个开关就完成飞行集成。

## 6. 当前边界及下一阶段

- 目前检查了图像/IMU/估计器消息持续更新，**没有证明视觉约束实际被接受**。OpenVINS 在弱纹理下仍可惯性传播，`poseimu` 更新和图像未断流都不能作为可靠定位标志。飞行前需补特征/更新接受状态、可观测性及协方差的健康判断。
- 普通 ROS1 `nav_msgs/Odometry`→MAVROS 通道不携带完整的 VIO `reset_counter`/quality 语义。本实现不自动重启、重定位续接或空中重置原点；必须设计并验证失效退出、坐标连续性和 PX4 源切换。
- D455 下视 VIO 不保证全场景：高空地面视差小、暗光、强光、单色地面、水面、运动模糊均会退化。GNSS、气压计和待增加的测距器需分别在 PX4 中验证融合。测距器的量程和表面条件决定近地高度约束适用范围。
- MID-360 纯感知迁移尚未实施。现有 FAST-LIO 注册点云和旧规划入口依赖 LIO；即便新规划入口使用 MAVROS 里程计，现有点云转换仍依赖 LIO。关闭 FAST-LIO 前还需新增基于 VIO/PX4 位姿和雷达实测外参的时间同步、点云变换/必要去畸变链路，并验证规划坐标一致性。
- 本地已做离线数值测试、OpenCV 配置读取和 XML/Python 检查。机载 `catkin_make`、rostest、相机实时运行、MAVROS/PX4 接收与融合尚需在目标机器完成，未声称已通过。

回退：停止所有新 launch，再 `sudo systemctl start grampusdrone.service` 恢复原入口。本次没有改自启动和控制器，也没有保存新的飞控参数；如地面试验自行改过参数，应从事先保存的参数备份恢复并核对。
