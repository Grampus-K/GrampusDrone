# Jetson：补装 D455 SDK 与 ROS1 驱动

适用本工程的 Ubuntu 20.04 arm64 / ROS Noetic / JetPack r35.6 环境。前提：已按 [OpenCV 修复说明](openvins_jetson_opencv.md) 编译 `~/grampus_cv_bridge_ws`，OpenVINS 已编译通过且链接 OpenCV 4.5。当前 `rospack find realsense2_camera` 失败，SDK 的 pkg-config、dpkg 和用户目录源码检查均无结果。

本步骤补齐两层依赖：librealsense **v2.50.0** 与 realsense-ros **2.3.2**。这是上游 2.3.2 明确配套的版本组合，与本工程 launch 接口一致；不是最新版本，也不代表已在这台 Orin 的固件/内核组合上完成验证。先完成设备枚举和地面采流再评估稳定性。

SDK 使用 RSUSB 后端，不执行内核补丁脚本。上游将其定位为适用于原型验证的用户态路径，生产环境推荐原生后端；不能据此假定飞行实时性、IMU 时间戳和断流表现已合格。

源码放在用户目录，SDK 安装到独立的 `/opt/grampus/librealsense-2.50.0`。ROS 驱动放入已有 `grampus_cv_bridge_ws`，与新 cv_bridge 一起构建，不增加第五个 ROS 工作空间，也无需为此重编 OpenVINS。不要同时安装另一套 apt RealSense 驱动。每块命令成功后再继续；失败时保留完整输出。

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
git clone --depth 1 --branch v2.50.0 \
  https://github.com/realsenseai/librealsense.git librealsense-2.50.0
cd ~/grampus_vio_deps/librealsense-2.50.0
git describe --tags --exact-match
```

标签应为 `v2.50.0`。以下仅构建 SDK 和命令行工具，不构建图形 viewer、CUDA 或 Python 绑定；本链路使用 ROS C++ 驱动。

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/grampus/librealsense-2.50.0 \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_INSTALL_RPATH=/opt/grampus/librealsense-2.50.0/lib \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_TM2=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF \
  -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF &&
cmake --build build --parallel 2 &&
sudo cmake --install build
```

安装前不加 sudo 编译。`-j2` 用于限制并发，构建时间取决于机载机资源；若出现编译器被 Killed，检查内存后可改为 `--parallel 1`。本步骤不刷相机固件。

## 3. 安装 USB 权限规则并验证设备

```bash
cd ~/grampus_vio_deps/librealsense-2.50.0
sudo install -m 0644 --backup=numbered config/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules

PKG_CONFIG_PATH=/opt/grampus/librealsense-2.50.0/lib/pkgconfig \
  pkg-config --modversion realsense2
ldd /opt/grampus/librealsense-2.50.0/bin/rs-enumerate-devices \
  | grep -E 'realsense|not found'
```

版本应为 `2.50.0`，librealsense 应来自上述独立目录，无 `not found`。SDK 不在默认搜索前缀，故这里显式提供 PKG_CONFIG_PATH；以后直接运行裸的 pkg-config 查不到它，不等于安装失败。安装生成的 RPATH 用于找到 SDK，不添加全局 LD_LIBRARY_PATH 或系统库软链接。

现在将 D455 接回 USB3，使用普通用户执行：

```bash
/opt/grampus/librealsense-2.50.0/bin/rs-enumerate-devices -S
```

必须实际列出 D455；保留设备固件版本与 USB 信息。如果输出 `No device detected`，即使命令退出码为零也不算通过。先排查 USB、权限与设备识别，再进入 ROS；不要通过一直 sudo 启动相机掩盖权限问题。当前没有读取相机固件，不能声称这套旧 SDK 已适配任意固件版本。

## 4. 在现有依赖工作空间构建 ROS 驱动

为防 `.bashrc` 自动加载 GrampusDrone 导致依赖工作空间反向依赖主工程，先进入干净 Bash：

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM="${TERM:-xterm}" LANG=C.UTF-8 bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
printf '%s\n' "$CMAKE_PREFIX_PATH"
```

输出应只有 livox_ws 与系统 ROS，不含 GrampusDrone 或 grampus_cv_bridge_ws。以下 clone 目标目录必须不存在；若已存在先核对版本和修改。

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
  -Drealsense2_DIR=/opt/grampus/librealsense-2.50.0/lib/cmake/realsense2
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

驱动版本应为 `2.3.2`，相机包和 cv_bridge 应指向 grampus_cv_bridge_ws，openvins_bridge 应指向 GrampusDrone。动态库应使用 `/opt/grampus/librealsense-2.50.0` 的 SDK、新 overlay 的 cv_bridge 和 OpenCV 4.5，不能混入 4.2 或出现 `not found`。

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

另一个正常终端检查 `/vio_inputs/left/image_raw`、`/vio_inputs/right/image_raw`、`/vio_inputs/imu` 的频率以及 `/d455_input/ready`。按 [主说明第 2 节](openvins_d455.md#2-启动相机导出这一台-d455-的出厂参数) 继续导出参数。SDK 枚举成功、驱动能启动、双目/IMU 稳定采流是三个分别验证的环节。

## 依据与验证边界

- [realsense-ros 2.3.2 说明](https://github.com/realsenseai/realsense-ros/blob/2.3.2/README.md)：Noetic 支持与 SDK 2.50.0 配套关系。
- [SDK v2.50.0 Jetson 指南](https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/installation_jetson.md)：RSUSB、原生后端差异。该旧文档没有验证本机 Orin/r35.6 组合。
- [SDK 构建选项](https://github.com/realsenseai/librealsense/blob/v2.50.0/CMake/lrs_options.cmake) 与 [安装路径](https://github.com/realsenseai/librealsense/blob/v2.50.0/CMake/install_config.cmake)。
- 本文命令依据固定版本源码核对；编写端是 Windows，未执行 Jetson 编译、USB 连接或硬件测试。
