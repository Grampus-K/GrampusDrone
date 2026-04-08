#!/bin/bash

#gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash" &

#sleep 2

gnome-terminal --tab --title="lidar" -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash" &

sleep 2

gnome-terminal --tab --title="mission" -- bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0"; exec bash" &

sleep 5

gnome-terminal --tab --title="imu" -- bash -c "rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0;sleep 1;rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0;sleep 1; exec bash" &

sleep 2


gnome-terminal --tab --title="fastlio" -- bash -c "roslaunch fast_lio mapping_mid360.launch; exec bash" &
sleep 5

gnome-terminal --tab --title="mavros" -- bash -c "rosrun lio_to_mavros lio_to_mavros_node; exec bash" &

wait
