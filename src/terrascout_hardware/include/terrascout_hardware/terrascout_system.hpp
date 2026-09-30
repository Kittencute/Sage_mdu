#pragma once

#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "terrascout_hardware/can_bus.hpp"
#include "terrascout_hardware/cubemars_protocol.hpp"

namespace terrascout_hardware
{

class TerrascoutSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TerrascoutSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool send_zero_rpm_to_all_motors(const char * transition);

  std::string can_interface_;
  int gear_ratio_ = 0;
  int pole_pairs_ = 0;
  int max_motor_erpm_ = 0;
  double can_send_failure_timeout_ = 0.0;
  double feedback_stale_timeout_ = 0.0;
  std::optional<rclcpp::Time> send_failure_started_;

  CanBus can_bus_;

  struct JointData
  {
    std::string name;
    uint8_t can_id = 0;
    bool negate = false;
    double accumulated_position = 0.0;
    double velocity = 0.0;
    std::optional<rclcpp::Time> last_feedback_time;
  };
  std::vector<JointData> joints_;
};

}  // namespace terrascout_hardware
