
GrampusDrone
介绍

作者：柯胤丞 

运行环境： 机载电脑：orin nano 8g 
ubuntu版本：20.04 
飞控：pixhawk 6cmini 1.13版本px4固件（必须用1.13）

对比fast-drone250的改进： 
1、将雷达里程计数据直接发给px4，然后px4将雷达里程计作为观测，融合imu获得/mavros/local_position/odom，px4ctrl里面用的就是这个mavros给的里程计 
2、px4ctrl的状态机和控制器全部改了，现在遥控器可以作为最高优先级控制飞机，并且无需拨杆 
3、规划器更换为ego-planner2，并且删减掉所有的集群相关和航点模式，只保留单机避障和rviz打点，并且规划器新增大量注释，launch文件也更加清晰

安装教程

    安装mid360驱动和fastlio2雷达定位，可参考https://www.bilibili.com/opus/986664810984767490
    安装mavros sudo apt-get install ros-noetic-mavros sudo apt-get install ros-noetic-mavros-extras cd /opt/ros/noetic/lib/mavros sudo ./install_geographiclib_datasets.sh
    安装ceres和glog 解压3rd_party.zip压缩包 进入glog文件夹打开终端 sh autogen.sh && sh configure && make && sudo make install sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev 进入ceres文件夹打开终端 mkdir build cd build cmake .. sudo make -j4 sudo make install
    编译工作空间 catkin_make 注意：在编译的时候如果报错说找不到什么文件，那就需要首先编译fastlio，然后编译lidar_imu_init，最后再编译其他的

使用说明

    运行shfiles里面的脚本 sh ready_go.sh
    等到显示校准完成之后，就可以打开roslaunch px4ctrl run.launch
    最后再sh ego_launch.sh

