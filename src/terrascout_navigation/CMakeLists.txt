cmake_minimum_required(VERSION 3.8)
project(terrascout_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  terrascout_navigation/gnss_shim.py
  terrascout_navigation/lidar_filter.py
  terrascout_navigation/teleop_takeover.py
  terrascout_navigation/twist_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_urdf_fixposition
    test/test_urdf_fixposition.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_lidar_filter
    test/test_lidar_filter.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(
    test_teleop_takeover
    test/test_teleop_takeover.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
