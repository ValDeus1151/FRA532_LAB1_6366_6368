set(_AMENT_PACKAGE_NAME "carver_navigation")
set(carver_navigation_VERSION "0.0.0")
set(carver_navigation_MAINTAINER "atinfinity <dandelion1124@gmail.com>")
set(carver_navigation_BUILD_DEPENDS "gazebo_ros_pkgs" "sensor_msgs")
set(carver_navigation_BUILDTOOL_DEPENDS "ament_cmake")
set(carver_navigation_BUILD_EXPORT_DEPENDS "gazebo_ros_pkgs" "sensor_msgs")
set(carver_navigation_BUILDTOOL_EXPORT_DEPENDS )
set(carver_navigation_EXEC_DEPENDS "diff_drive_controller" "gazebo_ros2_control" "gazebo_ros" "joint_state_broadcaster" "joint_state_publisher" "nav2_bringup" "navigation2" "robot_state_publisher" "ros2_control" "ros2_controllers" "slam_toolbox" "topic_tools" "xacro" "gazebo_ros_pkgs" "sensor_msgs")
set(carver_navigation_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(carver_navigation_GROUP_DEPENDS )
set(carver_navigation_MEMBER_OF_GROUPS )
set(carver_navigation_DEPRECATED "")
set(carver_navigation_EXPORT_TAGS)
list(APPEND carver_navigation_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND carver_navigation_EXPORT_TAGS "<gazebo_ros gazebo_plugin_path=\"lib\"/>")
list(APPEND carver_navigation_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"models\"/>")
