#sudo chmod 777 /dev/ttyACM0 & sleep 4;
#roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0" & sleep 8;
#rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
#rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
#roslaunch livox_ros_driver2 msg_MID360.launch & sleep 4;
#roslaunch fast_lio mapping_mid360.launch

roscore & sleep 2;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 2;
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0" & sleep 5;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch fast_lio mapping_mid360.launch & sleep 5;
rosrun lio_to_mavros lio_to_mavros_node

wait;

