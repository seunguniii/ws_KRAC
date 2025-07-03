cmake_minimum_required(VERSION 3.8)
project(transition_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(goto_setpoint_transition src/goto_setpoint_transition_at_arrival.cpp)
add_executable(trajectory_setpoint_L1 src/trajectory_setpoint_L1.cpp)

ament_target_dependencies(goto_setpoint_transition rclcpp px4_msgs)
target_include_directories(goto_setpoint_transition PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(trajectory_setpoint_L1 rclcpp px4_msgs)
target_include_directories(trajectory_setpoint_L1 PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  goto_setpoint_transition
  trajectory_setpoint_L1
  DESTINATION lib/${PROJECT_NAME})

ament_package()
