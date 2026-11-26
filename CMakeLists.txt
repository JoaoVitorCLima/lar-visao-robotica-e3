cmake_minimum_required(VERSION 3.0.2)
project(multi_camera_robot_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cv_bridge
  gazebo_ros
  husky_description
  husky_gazebo
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/fuse_poses.py
  scripts/apriltag_pose_node.py
  scripts/triangulate_marker.py
  scripts/evaluate_multi_camera.py
  scripts/record_multi_camera.py
  scripts/report_multi_camera.py
  scripts/visual_patrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config models worlds husky_urdf_extras
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
