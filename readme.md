# 🚁 GrampusDrone

GrampusDrone 是一个针对无人机自主飞行与避障优化的开源项目。本项目在 `fast-drone250` 的基础上进行了深度定制与改进，集成了 MID360 激光雷达、FastLIO2 里程计以及 EGO-Planner2 局部规划器。

👤 **作者**：柯胤丞

---

## 🌟 核心改进 (对比 fast-drone250)

1. **更优的定位融合架构**
   将雷达里程计数据直接发送给 PX4，由 PX4 将其作为观测数据与 IMU 进行 EKF 融合，输出 `/mavros/local_position/odom`。`px4ctrl` 控制器直接使用该融合后的高质量里程计数据。
2. **重构的状态机与控制器 (`px4ctrl`)**
   全面重写了状态机与底层控制逻辑。现在的遥控器（RC）拥有**最高控制优先级**，随时接管飞机，且**无需拨杆**即可实现干预，大幅提升了飞行安全性。
3. **升级的局部规划器 (`ego-planner2`)**
   - 规划器升级为 `EGO-Planner-v2`。
   - 移除冗余的集群相关代码和航点模式，专注保留**单机避障**与 **RViz 目标点导航**功能，运行更轻量。
   - Launch 文件经过清理，逻辑更加清晰。
   - 源码中新增了大量中文注释，极大降低了二次开发的门槛。

---

## 💻 运行环境

| 组件 | 配置要求 |
| :--- | :--- |
| **机载电脑** | NVIDIA Jetson Orin Nano (8GB) |
| **操作系统** | Ubuntu 20.04 |
| **ROS 版本** | ROS Noetic |
| **飞控硬件** | Pixhawk 6C mini |
| **飞控固件** | **PX4 v1.13** ⚠️ *(必须严格使用该版本)* |

> **⚠️ 重要提示：** PX4 固件版本必须为 `1.13`，否则可能导致 MAVROS 接口不兼容或 EKF 融合异常。

---

## 🛠️ 安装教程

### 1. 安装 MID360 驱动与 FastLIO2 定位
请参考 B站教程进行前置配置：[点击查看视频教程](https://www.bilibili.com/opus/986664810984767490)

### 2. 安装 MAVROS
打开终端，依次执行以下命令安装 MAVROS 及相关地理库：
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
