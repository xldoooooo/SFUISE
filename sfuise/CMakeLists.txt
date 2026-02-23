cmake_minimum_required(VERSION 3.5)
project(sfuise)

# 清除 ROS1 catkin 遗留变量，避免未使用变量警告
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_unused_catkin_var "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cf_msgs REQUIRED)
find_package(isas_msgs REQUIRED)
find_package(sfuise_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED MODULE)

ament_package()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(EstimationInterface src/EstimationInterface.cpp)
ament_target_dependencies(EstimationInterface rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs cf_msgs isas_msgs sfuise_msgs)

add_executable(SplineFusion src/SplineFusion.cpp)
ament_target_dependencies(SplineFusion rclcpp std_msgs sensor_msgs geometry_msgs cf_msgs isas_msgs sfuise_msgs)
target_link_libraries(SplineFusion cholmod)

install(TARGETS EstimationInterface SplineFusion
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

set(CMAKE_BUILD_TYPE "Debug")
