#include "terrascout_hardware/terrascout_system.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace terrascout_hardware
{

constexpr int MAX_CAN_NODE_ID = 0xFF;

hardware_interface::CallbackReturn TerrascoutSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto require_param = [&](const std::string & key) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: read parameter '%s'; cause: parameter is missing",
        key.c_str());
      return {};
    }
    return it->second;
  };

  can_interface_ = require_param("can_interface");
  if (can_interface_.empty()) return hardware_interface::CallbackReturn::ERROR;

  auto require_positive_int = [&](const std::string & key, int & value) -> bool {
    const auto raw = require_param(key);
    if (raw.empty()) return false;
    try {
      value = std::stoi(raw);
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(
        get_logger(), "Initialize hardware failed: operation: parse parameter '%s'; cause: %s",
        key.c_str(), exc.what());
      return false;
    }
    if (value <= 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: validate parameter '%s'; cause: expected > 0, got "
        "%d",
        key.c_str(), value);
      return false;
    }
    return true;
  };
  auto require_positive_double = [&](const std::string & key, double & value) -> bool {
    const auto raw = require_param(key);
    if (raw.empty()) return false;
    try {
      value = std::stod(raw);
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(
        get_logger(), "Initialize hardware failed: operation: parse parameter '%s'; cause: %s",
        key.c_str(), exc.what());
      return false;
    }
    if (value <= 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: validate parameter '%s'; cause: expected > 0, got "
        "%f",
        key.c_str(), value);
      return false;
    }
    return true;
  };
  if (
    !require_positive_int("gear_ratio", gear_ratio_) ||
    !require_positive_int("pole_pairs", pole_pairs_) ||
    !require_positive_int("max_motor_erpm", max_motor_erpm_) ||
    !require_positive_double("can_send_failure_timeout", can_send_failure_timeout_) ||
    !require_positive_double("feedback_stale_timeout", feedback_stale_timeout_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: check command interfaces of joint '%s'; cause: "
        "expected exactly 1 velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: check state interfaces of joint '%s'; cause: "
        "expected 2 state interfaces (position, velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    JointData jd;
    jd.name = joint.name;
    auto can_it = joint.parameters.find("can_id");
    if (can_it == joint.parameters.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: read parameter 'can_id' of joint '%s'; cause: "
        "parameter is missing",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    int can_id = 0;
    try {
      can_id = std::stoi(can_it->second);
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: parse parameter 'can_id' of joint '%s'; cause: %s",
        joint.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (can_id < 0 || can_id > MAX_CAN_NODE_ID) {
      RCLCPP_FATAL(
        get_logger(),
        "Initialize hardware failed: operation: validate parameter 'can_id' of joint '%s'; "
        "cause: expected 0 to %d, got %d",
        joint.name.c_str(), MAX_CAN_NODE_ID, can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    jd.can_id = static_cast<uint8_t>(can_id);
    auto neg_it = joint.parameters.find("negate");
    if (neg_it != joint.parameters.end() && neg_it->second == "true") {
      jd.negate = true;
    }
    joints_.push_back(jd);
  }

  RCLCPP_INFO(
    get_logger(), "Initialized with %zu joints on %s (gear_ratio=%d, pole_pairs=%d)",
    joints_.size(), can_interface_.c_str(), gear_ratio_, pole_pairs_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TerrascoutSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring — opening CAN bus %s", can_interface_.c_str());

  std::vector<struct can_filter> telemetry_filters;
  for (const auto & j : joints_) {
    telemetry_filters.push_back(
      {cubemars::telemetry_can_id(j.can_id), cubemars::TELEMETRY_ID_MASK});
  }
  if (!can_bus_.open(can_interface_, telemetry_filters)) {
    RCLCPP_FATAL(
      get_logger(), "Configure hardware failed: operation: open CAN interface %s; cause: %s",
      can_interface_.c_str(), can_bus_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & j : joints_) {
    j.accumulated_position = 0.0;
    j.velocity = 0.0;
    j.last_feedback_time.reset();
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TerrascoutSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating — enabling motors");
  send_failure_started_.reset();

  for (auto & j : joints_) j.last_feedback_time.reset();
  if (!send_zero_rpm_to_all_motors("Activate hardware")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool TerrascoutSystem::send_zero_rpm_to_all_motors(const char * transition)
{
  bool all_sent = true;
  for (const auto & j : joints_) {
    if (can_bus_.send(cubemars::encode_rpm(j.can_id, 0))) continue;
    all_sent = false;
    RCLCPP_ERROR(
      get_logger(),
      "%s failed: operation: send speed command 0 eRPM to motor '%s' CAN ID 0x%02X; cause: %s",
      transition, j.name.c_str(), j.can_id, can_bus_.last_error().c_str());
  }
  return all_sent;
}

hardware_interface::CallbackReturn TerrascoutSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating — stopping motors");
  if (!send_zero_rpm_to_all_motors("Deactivate hardware")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TerrascoutSystem::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    get_logger(), "Hardware error in state '%s' — stopping motors", previous_state.label().c_str());
  send_zero_rpm_to_all_motors("Handle hardware error");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn TerrascoutSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (!can_bus_.is_open()) return hardware_interface::CallbackReturn::SUCCESS;
  RCLCPP_INFO(get_logger(), "Shutting down — stopping motors");
  if (!send_zero_rpm_to_all_motors("Shut down hardware")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TerrascoutSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  constexpr int MAX_FRAMES = 64;

  for (int count = 0; count < MAX_FRAMES; ++count) {
    struct can_frame rx;
    const auto received = can_bus_.recv(rx);
    if (received == CanBus::RecvResult::Empty) break;
    if (received == CanBus::RecvResult::Error) {
      RCLCPP_ERROR(
        get_logger(), "Read motor feedback failed: operation: receive CAN frame on %s; cause: %s",
        can_interface_.c_str(), can_bus_.last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }

    const auto j = std::find_if(joints_.begin(), joints_.end(), [&rx](const JointData & joint) {
      return rx.can_id == cubemars::telemetry_can_id(joint.can_id);
    });
    if (j == joints_.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "Read motor feedback failed: operation: match CAN frame ID 0x%08X to a motor; "
        "cause: not the telemetry frame ID of a configured motor",
        rx.can_id);
      return hardware_interface::return_type::ERROR;
    }
    const auto tel = cubemars::decode_telemetry(rx.data, rx.can_dlc, gear_ratio_);
    if (!tel) {
      RCLCPP_ERROR(
        get_logger(),
        "Read motor feedback failed: operation: decode telemetry of motor '%s' CAN ID 0x%02X; "
        "cause: expected %zu data bytes, got %u",
        j->name.c_str(), j->can_id, cubemars::TELEMETRY_DLC, rx.can_dlc);
      return hardware_interface::return_type::ERROR;
    }
    if (tel->error_code != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Read motor feedback failed: operation: check fault state of motor '%s' CAN ID 0x%02X; "
        "cause: motor reports error code %u (%s)",
        j->name.c_str(), j->can_id, tel->error_code, cubemars::describe_error(tel->error_code));
      return hardware_interface::return_type::ERROR;
    }

    double wheel_rpm = static_cast<double>(tel->motor_erpm) / (gear_ratio_ * pole_pairs_);
    double velocity_rad_s = (wheel_rpm * 2.0 * M_PI) / 60.0;
    if (j->negate) velocity_rad_s = -velocity_rad_s;
    j->velocity = velocity_rad_s;
    j->last_feedback_time = time;
  }
  for (auto & j : joints_) {
    j.accumulated_position += j.velocity * period.seconds();
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, j.velocity);
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, j.accumulated_position);

    if (!j.last_feedback_time) {
      j.last_feedback_time = time;
      continue;
    }
    const double stale_for = (time - *j.last_feedback_time).seconds();
    if (stale_for >= feedback_stale_timeout_) {
      RCLCPP_ERROR(
        get_logger(),
        "Read motor feedback failed: operation: receive telemetry for motor CAN ID 0x%02X; "
        "cause: feedback stale for %.3f s, limit %.3f s",
        j.can_id, stale_for, feedback_stale_timeout_);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TerrascoutSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  bool send_failed = false;
  uint8_t failed_motor_id = 0;
  std::string failure_cause;
  for (auto & j : joints_) {
    double cmd_velocity = get_command<double>(j.name + "/" + hardware_interface::HW_IF_VELOCITY);

    if (!std::isfinite(cmd_velocity)) {
      cmd_velocity = 0.0;
    }
    if (j.negate) cmd_velocity = -cmd_velocity;

    const int32_t erpm = static_cast<int32_t>(std::clamp(
      cmd_velocity * (60.0 / (2.0 * M_PI)) * pole_pairs_ * gear_ratio_,
      -static_cast<double>(max_motor_erpm_), static_cast<double>(max_motor_erpm_)));

    auto frame = cubemars::encode_rpm(j.can_id, erpm);
    if (!can_bus_.send(frame)) {
      if (!send_failed) {
        failed_motor_id = j.can_id;
        failure_cause = can_bus_.last_error();
      }
      send_failed = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Write motor command failed: operation: send speed command %d eRPM to motor '%s' "
        "CAN ID 0x%02X; cause: %s",
        erpm, j.name.c_str(), j.can_id, can_bus_.last_error().c_str());
    }
  }
  if (!send_failed) {
    send_failure_started_.reset();
    return hardware_interface::return_type::OK;
  }
  if (!send_failure_started_) {
    send_failure_started_ = time;
    return hardware_interface::return_type::OK;
  }
  const double failed_for = (time - *send_failure_started_).seconds();
  if (failed_for >= can_send_failure_timeout_) {
    RCLCPP_ERROR(
      get_logger(),
      "Write motor command failed: operation: send CAN frame for motor 0x%02X; "
      "cause: %s; failure persisted for %.3f s, limit %.3f s",
      failed_motor_id, failure_cause.c_str(), failed_for, can_send_failure_timeout_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace terrascout_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(terrascout_hardware::TerrascoutSystem, hardware_interface::SystemInterface)
