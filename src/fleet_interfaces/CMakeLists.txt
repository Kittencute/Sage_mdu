cmake_minimum_required(VERSION 3.8)
project(fleet_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(FLEET_INTERFACES_MSG_FILES
  msg/BoundaryEntry.msg
  msg/UnitDescription.msg
  msg/MissionCommand.msg
  msg/MissionPlan.msg
  msg/MissionState.msg
  action/ExecuteMission.action
  srv/ControlMission.srv
  srv/GetKeepoutZone.srv
  srv/SetKeepoutZone.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${FLEET_INTERFACES_MSG_FILES}
  DEPENDENCIES geographic_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
