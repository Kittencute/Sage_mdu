#include <aeroscout_mission/mission.hpp>
#include <aeroscout_mission/mission_server.hpp>
#include <cstdlib>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <string>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string operation = "start mission node";
  try {
    auto node = std::make_shared<rclcpp::Node>("mission_server");
    aeroscout_mission::AeroscoutMission mission(node);
    const aeroscout_mission::AerialMissionServer server(node, mission);
    operation = "run mission node";
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mission_server"), "%s failed: cause: %s", operation.c_str(),
      error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
