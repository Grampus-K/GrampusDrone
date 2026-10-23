cmake_minimum_required(VERSION 3.0.2)
project(openvins_bridge)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs sensor_msgs geometry_msgs
  std_msgs mavros_msgs tf2_ros dynamic_reconfigure message_filters)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/bridge_node.py scripts/export_factory_calibration.py
  scripts/prepare_calibration.py scripts/camera_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_core.py)
  catkin_install_python(PROGRAMS test/test_bridge_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/bridge.test)
endif()
