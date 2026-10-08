cmake_minimum_required(VERSION 3.0.2)
project(lio_cloud_to_mavros)

find_package(catkin REQUIRED COMPONENTS mavros_msgs nav_msgs rospy sensor_msgs std_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS mavros_msgs nav_msgs rospy sensor_msgs std_msgs)

catkin_install_python(PROGRAMS scripts/registered_cloud_to_mavros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_geometry.py)
  catkin_add_nosetests(test/test_converter.py)
endif()
