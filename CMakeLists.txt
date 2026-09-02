cmake_minimum_required(VERSION 3.8)
project(mavtech_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(esekf_node
  src/esekf_node.cpp
)

add_executable(tf_broadcaster
  src/tf_broadcaster.cpp
)

add_executable(odom_sim_converter
  src/odom_sim_converter.cpp
)

add_executable(esekf_interface
  src/esekf_interface.cpp
)

target_link_libraries(esekf_interface
  Eigen3::Eigen
)

ament_target_dependencies(esekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  tf2
  tf2_eigen
  Eigen3
)

ament_target_dependencies(tf_broadcaster
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  tf2
  tf2_eigen
  Eigen3
)

ament_target_dependencies(odom_sim_converter
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  tf2
  tf2_eigen
  Eigen3
)

ament_target_dependencies(esekf_interface
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  px4_msgs
  tf2
  tf2_eigen
  Eigen3
)

target_include_directories(tf_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(odom_sim_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS odom_sim_converter
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(esekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS esekf_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS esekf_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
