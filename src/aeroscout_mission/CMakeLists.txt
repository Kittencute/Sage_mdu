cmake_minimum_required(VERSION 3.8)
project(aeroscout_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fleet_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_executable(mission_server
  src/mission_server.cpp
)

target_include_directories(mission_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mission_server PUBLIC
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

ament_target_dependencies(mission_server PUBLIC
  std_msgs
  rclcpp_action
  fleet_interfaces
  px4_msgs
)

install(TARGETS mission_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  aeroscout_mission/mission_progress.py
  aeroscout_mission/px4_health_watch.py
  aeroscout_mission/odom_converter.py
  aeroscout_mission/battery_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
