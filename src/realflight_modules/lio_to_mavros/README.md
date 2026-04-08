# lio_to_mavros

#### 介绍
fast-lio2里程计传给mavros的功能包

#### 使用说明

- 放在工作空间的src下就可以了，
- 然后安装sudo apt install ros-noetic-mavros-extras  这样才有mavros/vision_pose/pose话题
- 启动好fastlio2，并设置好ekf2 aid mask和ekf2 hgt mode参数
- 输入rosrun lio_to_mavros lio_to_mavros_node开启这个节点

