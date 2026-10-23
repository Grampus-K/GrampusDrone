# Jetson：补装 D455 SDK 与 ROS1 驱动

适用本工程的 Ubuntu 20.04 arm64 / ROS Noetic / JetPack r35.6 环境，用于首次安装或升级 D455 驱动依赖。前提：已按 [OpenCV 修复说明](openvins_jetson_opencv.md) 编译 `~/grampus_cv_bridge_ws`，OpenVINS 已编译通过且链接 OpenCV 4.5。

**固件兼容性修正：本机 D455 固件为 05.16.00.01，不能继续使用此前文档的 SDK 2.50.0。** 官方 SDK 2.55.1 发布说明指出，固件 5.16.0.1 将 gyro/accel 数据改为 32 位，旧 SDK 会错误解析 IMU。枚举成功不代表采流或 IMU 正确。

本步骤使用独立安装的 librealsense **v2.55.1**，保留 ROS1 realsense-ros **2.3.2** 的 launch 接口并针对新 SDK 重新编译。注意：2.3.2 上游发布时配套的是 SDK 2.50.0；这里是针对当前固件的源码重编方案，不是上游已验证的官方配套组合。本地已核对 CMake 最低版本要求及涉及的 SDK 接口，尚未执行 Jetson 编译和硬件验证。编译失败时保留第一条错误，不通过替换库软链接强行运行。

如果已经安装此前的 2.50.0：停止 launch 和整机服务后，从第 2 节构建到新的 2.55.1 目录，再做第 3、4 节。保留旧目录；已有且未修改的 realsense-ros 2.3.2 无需再次 clone，只需执行第 4 节的 catkin_make。无需重编 OpenVINS，也不更改相机固件。

SDK 使用 RSUSB 后端，不执行内核补丁脚本。上游将其定位为适用于原型验证的用户态路径，生产环境推荐原生后端；不能据此假定飞行实时性、IMU 时间戳和断流表现已合格。

源码放在用户目录，SDK 安装到独立的 `/opt/grampus/librealsense-2.55.1`。ROS 驱动放入已有 `grampus_cv_bridge_ws`，与新 cv_bridge 一起构建，不增加第五个 ROS 工作空间，也无需为此重编 OpenVINS。不要同时安装另一套 apt RealSense 驱动。每块命令成功后再继续；失败时保留完整输出。

## 1. 安装构建依赖

保持无人机在地面未解锁，停止整机服务；构建和安装 udev 规则期间先拔下 D455。

```bash
sudo systemctl daemon-reload
sudo systemctl stop grampusdrone.service
systemctl status grampusdrone.service --no-pager -l
```

确认服务为 `inactive (dead)` 后继续。

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config \
  libusb-1.0-0-dev libudev-dev libssl-dev \
  ros-noetic-ddynamic-reconfigure ros-noetic-diagnostic-updater \
  ros-noetic-nodelet ros-noetic-image-transport ros-noetic-tf \
  ros-noetic-message-generation ros-noetic-roscpp \
  ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-std-srvs
```

本步骤不安装 librealsense2-dkms，不升级 OpenCV。若 apt 提议移除/降级 NVIDIA OpenCV，或解析依赖失败，不继续确认。

## 2. 构建 SDK

以下源码和安装目录应尚未存在；已存在时先核对内容，不覆盖已有安装或本地修改。

```bash
mkdir -p ~/grampus_vio_deps
cd ~/grampus_vio_deps
git clone --depth 1 --branch v2.55.1 \
  https://github.com/realsenseai/librealsense.git librealsense-2.55.1
cd ~/grampus_vio_deps/librealsense-2.55.1
git describe --tags --exact-match
```

标签应为 `v2.55.1`。以下仅构建 SDK 和命令行工具，不构建图形 viewer、CUDA 或 Python 绑定；本链路使用 ROS C++ 驱动。

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/grampus/librealsense-2.55.1 \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_INSTALL_RPATH=/opt/grampus/librealsense-2.55.1/lib \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF \
  -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF &&
cmake --build build --parallel 2 &&
sudo cmake --install build
```

安装前不加 sudo 编译。`-j2` 用于限制并发，构建时间取决于机载机资源；若出现编译器被 Killed，检查内存后可改为 `--parallel 1`。本步骤不刷相机固件。

## 3. 安装 USB 权限规则并验证设备

```bash
cd ~/grampus_vio_deps/librealsense-2.55.1
sudo install -m 0644 --backup=numbered config/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules

PKG_CONFIG_PATH=/opt/grampus/librealsense-2.55.1/lib/pkgconfig \
  pkg-config --modversion realsense2
ldd /opt/grampus/librealsense-2.55.1/bin/rs-enumerate-devices \
  | grep -E 'realsense|not found'
```

版本应为 `2.55.1`，librealsense 应来自上述独立目录，无 `not found`。SDK 不在默认搜索前缀，故这里显式提供 PKG_CONFIG_PATH；以后直接运行裸的 pkg-config 查不到它，不等于安装失败。安装生成的 RPATH 用于找到 SDK，不添加全局 LD_LIBRARY_PATH 或系统库软链接。

现在将 D455 接回 USB3，使用普通用户执行：

```bash
/opt/grampus/librealsense-2.55.1/bin/rs-enumerate-devices -S
/opt/grampus/librealsense-2.55.1/bin/rs-enumerate-devices
```

必须实际列出 D455；保留设备固件版本与 USB 信息。如果输出 `No device detected`，即使命令退出码为零也不算通过。先排查 USB、权限与设备识别，再进入 ROS；不要通过一直 sudo 启动相机掩盖权限问题。固件 5.16.0.1 应使用这里的新 SDK。执行不带 `-S` 的 `rs-enumerate-devices` 还会列出传感器支持的采样档位，保留此输出；本机旧驱动日志实际选中 gyro 400 / accel 100 Hz，250 Hz 不支持。

## 4. 在现有依赖工作空间构建 ROS 驱动

为防 `.bashrc` 自动加载 GrampusDrone 导致依赖工作空间反向依赖主工程，先进入干净 Bash：

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM="${TERM:-xterm}" LANG=C.UTF-8 bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
printf '%s\n' "$CMAKE_PREFIX_PATH"
```

输出应只有 livox_ws 与系统 ROS，不含 GrampusDrone 或 grampus_cv_bridge_ws。以下 clone 仅用于首次安装，目标目录必须不存在。此前已安装 2.3.2 时，先用 `git -C ~/grampus_cv_bridge_ws/src/realsense-ros status --short` 和 `git -C ~/grampus_cv_bridge_ws/src/realsense-ros describe --tags --exact-match` 确认无修改且标签为 2.3.2，然后跳过 clone 两条命令，直接从 `cd ~/grampus_cv_bridge_ws` 开始重编。

```bash
cd ~/grampus_cv_bridge_ws/src
git clone --depth 1 --branch 2.3.2 \
  https://github.com/realsenseai/realsense-ros.git
cd ~/grampus_cv_bridge_ws
catkin_make -j2 --force-cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -Drealsense2_DIR=/opt/grampus/librealsense-2.55.1/lib/cmake/realsense2 \
  -DCMAKE_BUILD_RPATH=/opt/grampus/librealsense-2.55.1/lib \
  -DCMAKE_INSTALL_RPATH=/opt/grampus/librealsense-2.55.1/lib
```

OpenCV_DIR 来自这台 Jetson 已验证的 `dpkg -L libopencv-dev` 输出，其他机器不要照抄。该工作空间同时编译 cv_bridge 与相机驱动，主工程的 build/devel 不需要删除。

成功后在同一 Bash 中检查：

```bash
source ~/grampus_cv_bridge_ws/devel/setup.bash
source ~/GrampusDrone/devel/setup.bash
rospack find realsense2_camera
rosversion realsense2_camera
rospack find cv_bridge
rospack find openvins_bridge
ldd ~/grampus_cv_bridge_ws/devel/lib/librealsense2_camera.so \
  | grep -E 'realsense|opencv|cv_bridge|not found'
```

驱动版本应为 `2.3.2`，相机包和 cv_bridge 应指向 grampus_cv_bridge_ws，openvins_bridge 应指向 GrampusDrone。动态库应使用 `/opt/grampus/librealsense-2.55.1` 的 SDK、新 overlay 的 cv_bridge 和 OpenCV 4.5，不能混入 4.2 或出现 `not found`。

## 5. 固定终端环境并返回相机步骤

编辑 `.bashrc` 中已有的 source 行，顺序必须是下面四行，不再末尾追加另一份。cv_bridge 工作空间最后加载会恢复它的 underlay，导致主工程的包在新终端中消失。

```bash
source /opt/ros/noetic/setup.bash
source /home/orin/livox_ws/devel/setup.bash
source /home/orin/grampus_cv_bridge_ws/devel/setup.bash
source /home/orin/GrampusDrone/devel/setup.bash
```

退出上述干净 Bash，打开正常新终端（图像查看工具需要正常桌面环境），确认整机服务保持停止后执行：

```bash
rospack find realsense2_camera
rospack find openvins_bridge
roslaunch openvins_bridge d455.launch
```

另一个正常终端检查 `/vio_inputs/left/image_raw`、`/vio_inputs/right/image_raw`、`/vio_inputs/imu` 的频率以及 `/d455_input/ready`。若无数据，先按下一节检查原始话题。稳定采流、ready 为 true 且没有标定读取失败警告后，才按 [主说明第 2 节](openvins_d455.md#2-启动相机导出这一台-d455-的出厂参数) 继续导出参数。SDK 枚举成功、驱动能启动、双目/IMU 稳定采流是三个分别验证的环节。

## 6. launch 启动但无图像/IMU

本机问题日志：驱动 Built/Running with LibRealSense 2.50.0、FW 05.16.00.01，在 `setupStreams...` 后提示 IMU Calibration is not available、control_transfer Resource temporarily unavailable，未看到 `RealSense Node Is Up!`。这表明启动流程尚未确认完成。固件/SDK 的 IMU 格式不兼容已确定；它是否也是无图像的唯一原因，仍需原始采流检查。单条 control_transfer 警告不能单独证明 USB 硬件损坏。

保持 launch 运行，另开正常终端（使用 .bashrc 的既有环境），依次执行；`timeout` 到期结束是预期行为：

```bash
rostopic list | grep -E '^/d455/|^/vio_inputs/|^/d455_input/'
timeout 10s rostopic hz /d455/infra1/image_rect_raw
timeout 10s rostopic hz /d455/infra2/image_rect_raw
timeout 10s rostopic hz /d455/imu
rosservice list | grep -E '/d455/.*set_parameters'
```

- 原始双目和 IMU 都无数据：先处理 SDK/固件兼容、驱动初始化或 USB 传输，不调整 OpenVINS 配置。
- 原始双目有数据、合并 IMU 无数据：检查受支持档位、固件/SDK 和 IMU 合并。2.3.2 驱动启用合并时不发布独立的 `/d455/gyro/sample` 与 `/d455/accel/sample`；需要先停止相机，再以 `unite_imu_method:=""` 启动驱动，才能用这两个话题测试。100 Hz accel 不是合并 IMU 的输出频率。
- 三路原始流都有数据但 `/vio_inputs/*` 无数据：检查 d455_input 日志中的 emitter 确认、时间戳、左右配对或锁存故障。`ready=False` 表示检查未通过，并不直接说明相机离线。不要绕过检查强制改为 true。

rqt 中可选择原始 `/d455/infra1/image_rect_raw`、`/d455/infra2/image_rect_raw` 查看驱动是否出图；预览检查通过后再选择 `/vio_inputs/left/image_raw` 等转发话题。

这些红外图像是 mono8，rqt 选择 raw 传输即可；`compressedDepth` 用于深度图像，订阅红外图像的 `/compressedDepth` 会触发格式错误。这种订阅错误与启用 IMU 后原始流无数据是两个不同问题。

更新 SDK 后，启动日志必须同时显示 **Built with / Running with LibRealSense v2.55.1**。若仍是 2.50.0，先核对上面的 CMake 路径和 ldd，不能仅依据 pkg-config 的版本判断 ROS 驱动已更新。

若新版仍在 setupStreams 停住：Ctrl-C 停止相机、确认整机服务保持 inactive，并关闭其他相机进程；拔插 D455，再用短 USB3 数据线直连 Orin 排除外部 hub/线材和供电问题后重新检查。保留完整日志。`Physical Port 2-1-2` 本身不能证明是外部 hub 故障。

如果新版仍提示 **IMU Calibration is not available, default intrinsic and extrinsic will be used**，暂不导出出厂标定；需要进一步检查 SDK 是否能读取该设备的工厂 IMU 参数。这个警告不等于只是“还没做 Kalibr”，不能把默认外参当成真实出厂参数。

### 6.1 emitter_enabled 参数缺失导致整个 launch 退出

本机 SDK 2.55.1 的后续日志出现以下因果链：

```text
rs2_get_option(... Emitter Enabled ...) get_xu(id=2) failed
DynamicReconfigureParameterException: don't know parameter: emitter_enabled
REQUIRED process [d455_input-4] has died!
Initiating shutdown!
```

驱动注册动态参数时需要读取设备选项；读取失败会跳过该选项，因此服务存在但不包含 `emitter_enabled`。此前 camera_guard 未处理配置异常，而该节点是 required，导致整组 launch 被关闭。这并不是 launch 文件丢失或 Python 找不到包。

camera_guard 现已捕获参数缺失、服务异常和配置未确认等启动失败，锁存故障、持续发布 `ready=False`，不转发图像或 IMU，让驱动和诊断保持运行。晚到的配置回调不能自动解除故障；修复设备通信后需重启 launch。该修正不代表 USB 问题已经解决，也不通过删除 required、伪造 ROS 参数或绕过投射器检查恢复 VIO。

日志中的 `control_transfer ... error: Success, number: 0` 不代表传输成功：SDK 在 libusb 返回负值时进入错误分支，但文本取自 errno，可能仍显示 Success。需要结合前面的 get_xu 错误判断，而不能忽略它。

### 6.2 使用纯双目诊断入口隔离问题

先在所有相机 launch 终端 Ctrl-C，等待退出，并停止整机服务。无人机保持未解锁：

```bash
sudo systemctl stop grampusdrone.service
systemctl is-active grampusdrone.service
pgrep -af 'realsense2_camera|realsense-viewer|rs-enumerate-devices|d455.launch|d455_ir_test.launch'
```

服务应为 inactive；如果还有相关相机进程，先定位其启动终端并正常关闭，不直接 kill 所有 ROS/nodelet 进程。没有其他进程使用相机后，将 D455 USB 完全拔下约 10 秒再接回，优先短 USB3 数据线直连 Orin。此操作用于恢复设备通信，不能仅凭恢复后可用就断定是线材故障。

更新工程后，使用独立诊断入口：

```bash
cd ~/GrampusDrone
git pull --ff-only origin master
roslaunch openvins_bridge d455_ir_test.launch 2>&1 | tee ~/d455_ir_test.log
```

现有 catkin devel 布局直接读取源码 launch/Python，已有 openvins_bridge 的机器无需为本次修正重编 SDK 或主工程。纯双目入口关闭 IMU，不启动 camera_guard、OpenVINS 或 PX4 桥接；不确认投射器状态，只用于排查原始图像采流，不能直接用于标定或定位。

另一个正常终端检查：

```bash
timeout 10s rostopic hz /d455/infra1/image_rect_raw
timeout 10s rostopic hz /d455/infra2/image_rect_raw
```

纯双目测试不发布 `/vio_inputs/*` 和 `/d455_input/ready`，rqt 也请选择以上原始图像话题。若双目能稳定出图，再针对 IMU/双目组合和检查节点分开测试；若纯双目也有 get_xu 错误或不出图，收集 USB 拓扑和内核日志继续检查设备、传输和驱动后端：

```bash
lsusb -t
sudo dmesg --ctime | tail -n 80
```

避免 SDK 设备工具、RealSense Viewer 和多个 ROS 相机驱动同时访问同一台相机；rqt_image_view 订阅 ROS 图像可以与相机 launch 一起运行。正常定位入口仍是 `d455.launch`，切回前先停止纯双目测试。

### 6.3 纯双目正常，但启用 IMU 后所有流停止

本机后续实测：纯双目两路约 30 Hz；直接启动 realsense2_camera（不含 camera_guard/OpenVINS），同时启用 gyro/accel 后原始双目和 IMU 都无消息，关闭 IMU 合并也没有恢复。旧日志持续出现 `get_xu(ctrl=1)`、`get_xu(id=7)` 读取失败。因此不能继续把问题归到 `/vio_inputs` 转发或仅仅归到 IMU 插值；应先检查 ROS 之外能否采流。也不能据此认定物理 IMU 已损坏。

工程提供独立 SDK 诊断源码与脚本，直接链接已经安装的 `/opt/grampus/librealsense-2.55.1`。不需要重新构建 SDK、catkin 或安装 Python RealSense 绑定。脚本只构建一个小程序，记录设备、USB 拓扑、实际链接库、每个传感器 open/start 的前后位置和 10 秒内累计帧数。它不写相机选项、出厂标定、固件或系统配置；打开传感器会临时开始采流。整个采流进程最长 45 秒，必要时再给 5 秒强制结束，防止初始化或退出卡住。

先 Ctrl-C 关闭所有相机 launch 和设备工具，保持整机服务停止。完全拔下 D455 约 10 秒后接回，沿用纯双目测试成功的 USB 线和接口，避免一次改变多个变量。然后执行：

```bash
cd ~/GrampusDrone
git pull --ff-only origin master
bash src/realflight_modules/openvins_bridge/diagnostics/run_d455_sdk_probe.sh all 341522300469
```

上述序列号属于当前这台 D455，换相机时使用实际序列号；只连接一台 RealSense 时也可省略。命令运行时不要启动相机 ROS 节点或 Viewer。`.bashrc` 的 ROS source 顺序不影响该独立诊断，脚本仅在诊断子进程中清除 LD_LIBRARY_PATH/LD_PRELOAD，以使用指定 SDK。

`all` 请求与本次 ROS 测试相同的 gyro 400 Hz、accel 100 Hz、双目 848×480@30 Y8，按模块名排序启动（Motion Module 在 Stereo Module 前，与 ROS 2.3.2 一致）。不支持某档位时明确报错，不能静默回退。若 Motion Module 的 open/start 卡住，程序可能还未开始启动双目，因此四路都无数据不能推出四个传感器都坏了。

| 报告内容 | 可以得出的结论 / 下一步 |
| --- | --- |
| 最后一条 STAGE 是 Motion Module 的 `open begin` 或 `start begin`，随后超时 | 独立 SDK 也卡在运动传感器初始化；优先查 SDK/RSUSB、USB 访问和设备固件交互，不修改 OpenVINS。 |
| open/start 都完成，但某些 COUNTS 始终为 0 | 独立 SDK 已启动接口但没收到对应帧；保留具体流和 SDK 错误，继续查采流链路。 |
| `RECEIVED_ALL_REQUESTED_STREAMS` 且退出码 0 | 这一次独立 SDK 四路均收到帧；接下来比较 ROS 驱动设置和启动流程。仅证明有帧，不证明帧率、时间戳、同步和标定合格。 |
| Missing exact profile / SDK headers and runtime differ / 编译失败 | 测试尚未进入有效采流阶段；先处理报告中明确的档位、版本或构建错误。 |

完整报告保存在终端打印的 `~/grampus_vio_deps/d455_diagnostics/all_XXXXXXXX/report.log`，请提供整个文件，不再只截最后 100 行。`PROBE_EXIT_CODE=124` 表示超时；137 表示被 SIGKILL 结束，需结合最后阶段判断。工具也支持 `imu` 和 `ir` 模式，供后续有针对性地隔离测试；某次卡住后先关闭进程、拔插恢复设备，不自动连续运行多组测试。

若需要补充此前 ROS 的初始化过程，应查看新运行自己的日志开头：

```bash
sed -n '1,220p' ~/d455_unmerged_imu_test.log
```

`d455_stereo_imu_test.log` 是之前合并模式的日志，不能替代 `d455_unmerged_imu_test.log`。反复出现的 `time_diff_keeper` 表示读取设备时钟失败，不足以证明主机系统时间错误；暂不以关闭时间同步选项、关闭错误轮询或降级固件来掩盖通信失败。

该诊断工具在编写端未连接 D455，需以上述 Orin 实测报告判断结果；不能把脚本或源码检查通过当作硬件采流已修复。

## 依据与验证边界

- [SDK 2.55.1 官方发布说明](https://github.com/realsenseai/librealsense/wiki/Release-Notes#release-2551)：固件 5.16.0.1 的 IMU 数据格式变更及旧 SDK 不兼容。
- [realsense-ros 2.3.2 说明](https://github.com/realsenseai/realsense-ros/blob/2.3.2/README.md)：ROS Noetic 与原发布的 SDK 2.50.0 配套关系；[CMake](https://github.com/realsenseai/realsense-ros/blob/2.3.2/realsense2_camera/CMakeLists.txt) 对 SDK 使用最低版本要求。
- [SDK v2.55.1 Jetson 指南](https://github.com/realsenseai/librealsense/blob/v2.55.1/doc/installation_jetson.md)：RSUSB 与原生后端的差异。
- [SDK 构建选项](https://github.com/realsenseai/librealsense/blob/v2.55.1/CMake/lrs_options.cmake) 与 [安装路径](https://github.com/realsenseai/librealsense/blob/v2.55.1/CMake/install_config.cmake)。
- [ROS 驱动动态选项注册](https://github.com/realsenseai/realsense-ros/blob/2.3.2/realsense2_camera/src/base_realsense_node.cpp) 与 [SDK USB 错误日志实现](https://github.com/realsenseai/librealsense/blob/v2.55.1/src/libusb/messenger-libusb.cpp)。
- 本文命令依据固定版本源码核对；编写端是 Windows，未执行 Jetson 编译、USB 连接或硬件测试。
