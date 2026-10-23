# Jetson：OpenVINS 的 OpenCV / cv_bridge 依赖修复

适用已确认的环境：Ubuntu 20.04 arm64 / JetPack r35.6，NVIDIA `libopencv-dev` 为 `4.5.4-8-g3e4c170df4`，Ubuntu 的 `libopencv-contrib-dev` 只提供 4.2，系统 `/opt/ros/noetic/lib/libcv_bridge.so` 链接 4.2，且现有头文件目录没有 `opencv2/aruco.hpp`。

这里有两个问题：contrib 4.2 开发包不能配到 NVIDIA 4.5.4；OpenVINS 若使用 4.5.4，也不能混用链接 4.2 的 cv_bridge。保留 NVIDIA OpenCV，用独立 catkin overlay 重编译 cv_bridge，并关闭 OpenVINS 的 ArUco 编译支持。不卸载或降级 NVIDIA 包，不覆盖 `/opt/ros/noetic`，不创建跨版本库软链接。

固定的 OpenVINS 源码在 `ov_core/CMakeLists.txt` 和 `ov_msckf/CMakeLists.txt` 中提供 `ENABLE_ARUCO_TAGS`，`TrackAruco.h/.cpp` 用该宏保护 ArUco 代码。**编译时 `-DENABLE_ARUCO_TAGS=OFF` 加上运行时 `use_aruco: false`，即可使用普通双目特征跟踪而不需要 contrib。** 仅修改运行时配置不会关闭默认开启的编译支持。本工程生成配置采用的上游 rs_d455 模板已是 `use_aruco: false`；后续保持该值。

以下命令在机载 Ubuntu 执行，替代主说明第 1 节。尚未在机载机验证；每一步成功后再执行下一步，失败时保留完整输出。

## 1. 安装基础依赖

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config python3-dev \
  python3-numpy python3-yaml python3-nose python3-opencv python3-catkin-pkg \
  libboost-all-dev libeigen3-dev libceres-dev \
  ros-noetic-rostest ros-noetic-dynamic-reconfigure ros-noetic-message-filters \
  ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-image-transport
```

这里不安装 `libopencv-contrib-dev`，也不替换现有 OpenCV 开发包。apt 若仍报冲突，先检查完整输出。

## 2. 在独立 overlay 编译 cv_bridge

使用未加载原工程 devel 的干净终端。若 `.bashrc` 自动 source 原工程，打开新终端也会带入旧环境；可先用下面命令进入干净 Bash（`exit` 返回原终端）：

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM="${TERM:-xterm}" LANG=C.UTF-8 bash --noprofile --norc
```

在此干净 Bash 中加载 ROS 和 Livox，确认 Livox 的 setup 没有重新带入原工程：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
printf '%s\n' "$CMAKE_PREFIX_PATH"
pkg-config --modversion opencv4
dpkg -L libopencv-dev | grep '/OpenCVConfig.cmake$'
```

`CMAKE_PREFIX_PATH` 不应包含 `~/GrampusDrone/devel`，OpenCV 应为 4.5.4。最后一条必须找到 NVIDIA 开发包提供的配置文件；没有输出时先停止。接着在同一 Bash 中保存其目录：

```bash
grampus_opencv_config="$(dpkg -L libopencv-dev | grep '/OpenCVConfig.cmake$' | head -n 1)"
test -n "$grampus_opencv_config" && test -f "$grampus_opencv_config"
grampus_opencv_dir="${grampus_opencv_config%/*}"
```

下面使用全新的 `~/grampus_cv_bridge_ws`；目录已存在时先检查内容，不覆盖本地修改。源码标签为 Noetic 的 1.16.2，vision_opencv 同时包含 image_geometry，使它也使用同一 OpenCV。

```bash
mkdir -p ~/grampus_cv_bridge_ws/src
cd ~/grampus_cv_bridge_ws/src
git clone --depth 1 --branch 1.16.2 \
  https://github.com/ros-perception/vision_opencv.git
cd ~/grampus_cv_bridge_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DOpenCV_DIR="$grampus_opencv_dir"
source devel/setup.bash
rospack find cv_bridge
ldd devel/lib/libcv_bridge.so | grep -E 'opencv|not found'
```

`rospack find cv_bridge` 应指向 `~/grampus_cv_bridge_ws/src/vision_opencv/cv_bridge`。`ldd` 中 OpenCV 应全部为 4.5，不应出现 4.2 或 `not found`。系统 `/opt/ros/noetic/lib/libcv_bridge.so` 仍然链接 4.2 是正常的，本方案使用 overlay 的新库。无需全局修改 `LD_LIBRARY_PATH` 或运行 `ldconfig`。

apt 的 Python `cv2` 仍可能是 4.2，因此 Python 版本不能作为 C++ 验收依据。本次使用的是 C++ OpenVINS/cv_bridge 链路；本工程的 camera_guard 使用 ROS 消息转发，不通过 Python cv_bridge 转换图像。

## 3. 用这套 overlay 编译工程

在同一个已 source 新 overlay 的 Bash 中执行：

```bash
cd ~/GrampusDrone
python3 shfiles/fetch_openvins.py
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic \
  --skip-keys="libopencv-dev libopencv-contrib-dev"
catkin_make -j2 --force-cmake -DCATKIN_ENABLE_TESTING=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DENABLE_ARUCO_TAGS=OFF \
  -DOpenCV_DIR="$grampus_opencv_dir" \
  -Dcv_bridge_DIR="$HOME/grampus_cv_bridge_ws/devel/share/cv_bridge/cmake" \
  -Dimage_geometry_DIR="$HOME/grampus_cv_bridge_ws/devel/share/image_geometry/cmake"
source devel/setup.bash
ldd devel/lib/ov_msckf/run_subscribe_msckf | grep -E 'opencv|cv_bridge|not found'
catkin_make run_tests_openvins_bridge -j2
catkin_test_results
```

`--skip-keys` 跳过已由 NVIDIA 提供的 OpenCV，以及本次关闭的 contrib 功能；其他缺失依赖仍须解决。CMake 应显示 OpenCV 4.5.4 和 `DISABLING ARUCOTAG TRACKING!`。OpenVINS 可执行程序应加载 overlay 的 cv_bridge，OpenCV 库全部为 4.5，不能同时出现 4.2/4.5 或 `not found`。如仍混用，保留 CMake 输出与 `ldd`，排查传递依赖。

后续新增配置也必须保持 `use_aruco: false`，否则调用被禁用的标记跟踪器会退出。普通双目 VIO 不受该功能关闭影响；将来确需 ArUco 时再配置完整 contrib 和一致的依赖。

## 4. 预览时加载环境

每个新终端依次加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/grampus_cv_bridge_ws/devel/setup.bash
source ~/GrampusDrone/devel/setup.bash
rospack find cv_bridge
```

验证后回到 [主操作说明第 2 节](openvins_d455.md#2-启动相机导出这一台-d455-的出厂参数) 启动相机。此修复解决构建依赖，相机、VIO 精度和飞控融合仍需后续验证。

## 回退说明

原工程 devel/build 在第 3 节会按新依赖重编译，不能仅关闭终端就认定恢复了旧二进制。若需保留旧版本运行，先在独立工程 checkout 中完成第 3 节，并相应修改工程路径。本方案不修改现有自启动，也不覆盖系统 OpenCV/cv_bridge。
