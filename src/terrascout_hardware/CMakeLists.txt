cmake_minimum_required(VERSION 3.8)
project(terrascout_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(dep ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/can_bus.cpp
  src/cubemars_protocol.cpp
  src/terrascout_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface terrascout_hardware.xml)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  terrascout_hardware/battery_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cubemars_protocol test/test_cubemars_protocol.cpp)
  target_link_libraries(test_cubemars_protocol ${PROJECT_NAME})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tattu_bms test/test_tattu_bms.py)
endif()

ament_package()
