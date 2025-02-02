cmake_minimum_required(VERSION 3.0.2)

set (CMAKE_CXX_STANDARD 17)

project(f1tenth_verification)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs cv_bridge image_transport)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/verification/object_tracking_node/object_tracking_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/verification/collision_verification/collision_verification_ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/verification/collision_verification_plotter/live_probability_of_collision_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/verification/collision_verification_plotter/live_bounding_box_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/verification/collision_verification_plotter/live_probstar_projection_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/verification/collision_verification_plotter/live_semantic_segmentation_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

