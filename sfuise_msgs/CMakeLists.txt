cmake_minimum_required(VERSION 3.5)
project(sfuise_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Spline.msg"
  "msg/Knot.msg"
  "msg/Calib.msg"
  "msg/Offset.msg"
  "msg/Estimate.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()

