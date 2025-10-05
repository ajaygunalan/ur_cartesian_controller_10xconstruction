cmake_minimum_required(VERSION 3.8)
project(ur_cartesian_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# Find OSQP
find_package(osqp REQUIRED)
find_package(osqp_vendor REQUIRED)

# Manually find OsqpEigen (not always in standard cmake configs)
find_path(OSQP_EIGEN_INCLUDE_DIR
  NAMES OsqpEigen/OsqpEigen.h
  PATHS /usr/include /usr/local/include
)

if(NOT OSQP_EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR "OsqpEigen headers not found. Install: sudo apt install ros-${ROS_DISTRO}-osqp-vendor")
endif()

# Executable
add_executable(cartesian_controller
  src/cartesian_controller.cpp
)

target_include_directories(cartesian_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_EIGEN_INCLUDE_DIR}
)

ament_target_dependencies(cartesian_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  kdl_parser
  orocos_kdl
  osqp
  osqp_vendor
)

target_link_libraries(cartesian_controller
  osqp::osqp
  orocos-kdl
  /usr/local/lib/libOsqpEigen.so
)

install(TARGETS cartesian_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/demo_grid.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
