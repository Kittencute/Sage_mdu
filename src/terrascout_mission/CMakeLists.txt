cmake_minimum_required(VERSION 3.8)
project(terrascout_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(PROGRAMS
  terrascout_mission/mission_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_waypoint_geojson
    test/test_waypoint_geojson.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
