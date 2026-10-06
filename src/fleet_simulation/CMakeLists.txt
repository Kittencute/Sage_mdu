cmake_minimum_required(VERSION 3.8)
project(fleet_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

add_library(velocity_command_timeout SHARED src/velocity_command_timeout.cpp)
target_compile_features(velocity_command_timeout PUBLIC cxx_std_17)
target_link_libraries(velocity_command_timeout PRIVATE gz-sim::gz-sim gz-plugin::register)

install(TARGETS velocity_command_timeout LIBRARY DESTINATION lib)
install(PROGRAMS src/simulated_battery.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY terrain DESTINATION share/${PROJECT_NAME})

ament_package()
