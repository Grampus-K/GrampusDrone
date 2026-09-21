# GrampusDrone

基于 [FAST-Drone-250](https://github.com/ZJU-FAST-Lab/FAST-Drone-250) 和
[FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的单机无人机避障工程。

作者：柯胤丞

## 主要改动

- 将雷达里程计发送给 PX4，由 PX4 融合 IMU 后输出 `/mavros/local_position/odom`，`px4ctrl` 使用该里程计进行控制。
- 修改 `px4ctrl` 状态机和控制器，使遥控器具有最高控制优先级，且无需拨动模式开关。
- 将规划器更换为 `ego-planner2`，移除集群和航点模式，仅保留单机避障与 RViz 打点功能，并补充注释和启动文件说明。

## 运行环境

- 机载电脑：NVIDIA Orin NX（Orin Nano 也可参考）
- Ubuntu：20.04
- ROS：ROS 1 Noetic
- 飞控：Pixhawk 6C Mini，PX4 固件 1.13（必须使用 1.13）
- 传感器：Livox Mid-360（其他雷达需要相应修改配置）

## 安装依赖

### 1. 安装 Mid-360、Livox 驱动和 FAST-LIO2

安装方法可参考：

<https://www.bilibili.com/opus/986664810984767490>

确认 Livox 驱动工作空间已经编译，并在编译本工程前加载其环境：

```bash
source ~/livox_ws/devel/setup.bash
```

如果你的 Livox 工作空间路径不同，请替换上面的路径。

### 2. 安装 MAVROS

```bash
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

将当前用户加入 `dialout` 用户组，以永久获得 `/dev/ttyACM0` 等串口设备的访问权限：

```bash
sudo usermod -aG dialout "$USER"
```

执行后需要注销并重新登录，或重启系统，权限才会生效。

### 3. 安装 Ceres 及其依赖

工程自带的 `3rd_party.zip` 中包含 Ceres 2.0.0-rc1 和 glog 源码。在 Ubuntu 20.04 上建议直接使用系统提供的 glog 开发包，再从压缩包源码编译 Ceres：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libatlas-base-dev \
  libsuitesparse-dev
```

已经安装的依赖会被 `apt` 自动跳过。不要再单独安装固定版本的 `libcxsparse3.1.2`：这个包名在 Ubuntu 20.04 或部分 JetPack 软件源中可能不存在，`libsuitesparse-dev` 会提供 Ceres 所需的 SuiteSparse/CXSparse 开发文件。

安装 `libgoogle-glog-dev` 后通常不需要再编译压缩包中的 glog，以免 `/usr` 和 `/usr/local` 中出现两套 glog。解压 `3rd_party.zip` 后编译 Ceres：

```bash
cd ceres-solver-2.0.0rc1
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
```

如果压缩包解压后的 Ceres 目录不在当前路径，请先进入它所在的父目录，或将上面的目录名替换为实际路径。

## 编译工程

```bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash

catkin_make -j$(nproc)
```

当前工程已经为 FAST-LIO 和 LiDAR-IMU-Init 的自定义消息生成目标声明了正确的 CMake 依赖，正常情况下可以直接全量编译，不需要先单独编译这两个包。

## 运行

```bash
cd ~/GrampusDrone
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source devel/setup.bash

sh shfiles/ready_go.sh
```

等待校准完成后，再启动控制器和规划器：

```bash
roslaunch px4ctrl run_ctrl.launch
sh shfiles/ego_launch.sh
```

具体启动脚本和参数请根据实际硬件、雷达安装方向及 PX4 配置进行确认。

## OpenCV 版本提示（Orin NX）

在 Orin NX + Ubuntu 20.04 + ROS Noetic 环境中，JetPack 通常提供 OpenCV 4.5.4，而 ROS Noetic 自带的 `cv_bridge` 通常按 OpenCV 4.2 编译。全量链接时可能看到：

```text
libopencv_core.so.4.2 ... may conflict with libopencv_core.so.4.5
```

如果 `catkin_make` 成功完成且程序运行正常，可以暂时忽略该警告。不要直接删除或替换系统中的 OpenCV 库；如果以后出现运行时崩溃或图像处理异常，再统一 OpenCV 版本或重新编译 `cv_bridge`。

## Ubuntu 端 Git 操作

Windows 和 Ubuntu 使用同一个 GitHub 仓库。在 Ubuntu 开始修改前，先拉取 `master` 分支的最新内容：

```bash
cd ~/GrampusDrone
git status
git pull --rebase origin master
```

建议只在工作区没有未提交修改时执行 `git pull --rebase`。完成代码或配置修改后，按下面的流程提交并推送：

```bash
cd ~/GrampusDrone

# 查看改动，确认没有包含运行日志或其他生成文件
git status
git diff

# 按实际路径添加需要提交的文件，可以重复执行多次
git add <文件路径>

# 检查即将提交的内容
git diff --cached
git status

# 创建本地提交
git commit -m "简要说明本次修改"

# 推送前同步远端的新提交，然后上传到 GitHub
git pull --rebase origin master
git push origin master

# 确认本地与远端一致
git status
```

除非已经确认所有改动都需要提交，否则不要直接使用 `git add -A`。尤其要注意 `FAST_LIO/Log`、`LiDAR_IMU_Init/Log` 和 `LiDAR_IMU_Init/result` 中的运行或标定输出；如果这些文件不是本次修改内容，就不要加入提交。
