cmake_minimum_required(VERSION 3.5)
project(isas_msgs)

# 清除 ROS1 catkin 遗留变量，避免未使用变量警告
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_unused_catkin_var "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RTLSRange.msg"
  "msg/RTLSStick.msg"
  "msg/AnchorPosition.msg"
  "msg/Anchorlist.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()

