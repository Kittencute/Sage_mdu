#pragma once

#include <aeroscout_mission/mission.hpp>
#include <chrono>
#include <cmath>
#include <fleet_interfaces/action/execute_mission.hpp>
#include <fleet_interfaces/msg/mission_command.hpp>
#include <fleet_interfaces/msg/mission_plan.hpp>
#include <fleet_interfaces/msg/mission_state.hpp>
#include <fleet_interfaces/srv/control_mission.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

namespace aeroscout_mission
{

inline constexpr const char * kNoReferenceAltitude = "no global reference altitude from PX4 yet";

class AerialMissionServer
{
public:
  using ExecuteMission = fleet_interfaces::action::ExecuteMission;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  using MissionCommand = fleet_interfaces::msg::MissionCommand;
  using MissionState = fleet_interfaces::msg::MissionState;
  using ControlMission = fleet_interfaces::srv::ControlMission;
  using VehicleCommand = px4_msgs::msg::VehicleCommand;
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

  static constexpr double kReadyTimeoutS = 20.0;
  static constexpr double kActivationTimeoutS = 20.0;
  static constexpr double kArmDelayS = 1.0;
  static constexpr double kDeactivationTimeoutS = 20.0;

  static rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
  }

  static rcl_interfaces::msg::ParameterDescriptor describe(
    const std::string & description, double low, double high)
  {
    auto descriptor = describe(description);
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = low;
    range.to_value = high;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);
    return descriptor;
  }

  AerialMissionServer(const std::shared_ptr<rclcpp::Node> & node, AeroscoutMission & mission)
  : node_(node), mission_(mission), mode_id_(mission.modeId())
  {
    node_->declare_parameter<double>(
      "default_horizontal_velocity",
      describe("Horizontal speed of a mission leg, in metres per second", 0.1, 20.0));
    node_->declare_parameter<double>(
      "default_vertical_velocity",
      describe("Vertical speed of a mission leg, in metres per second", 0.1, 10.0));
    node_->declare_parameter<double>(
      "default_max_heading_rate",
      describe("Fastest turn rate about the vertical axis, in degrees per second", 0.0, 360.0));
    node_->declare_parameter<bool>(
      "force_arm", describe("Whether to arm even when PX4 preflight checks fail"));
    horizontal_velocity_ = node_->get_parameter("default_horizontal_velocity").as_double();
    vertical_velocity_ = node_->get_parameter("default_vertical_velocity").as_double();
    max_heading_rate_ = node_->get_parameter("default_max_heading_rate").as_double();
    force_arm_ = node_->get_parameter("force_arm").as_bool();
    if (!(horizontal_velocity_ > 0.0) || !(vertical_velocity_ > 0.0)) {
      throw std::runtime_error(
        "read speed limits failed: cause: default_horizontal_velocity " +
        std::to_string(horizontal_velocity_) + " and default_vertical_velocity " +
        std::to_string(vertical_velocity_) + " must both be positive");
    }

    state_pub_ = node_->create_publisher<MissionState>(
      "mission_state", rclcpp::QoS(1).reliable().transient_local());
    plan_pub_ = node_->create_publisher<fleet_interfaces::msg::MissionPlan>(
      "mission_plan", rclcpp::QoS(1).reliable().transient_local());
    command_pub_ = node_->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);
    status_sub_ = node_->create_subscription<VehicleStatus>(
      "fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
      [this](const VehicleStatus & msg) { status_ = msg; });
    local_position_sub_ = node_->create_subscription<VehicleLocalPosition>(
      "fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
      [this](const VehicleLocalPosition & msg) {
        if (!msg.z_global) {
          return;
        }
        const bool first = !reference_altitude_;
        reference_altitude_ = msg.ref_alt;
        if (first && !active_) {
          RCLCPP_INFO(node_->get_logger(), "ready for missions");
          publishIdle();
        }
      });

    server_ = rclcpp_action::create_server<ExecuteMission>(
      node_, "execute_mission",
      [this](const rclcpp_action::GoalUUID &, ExecuteMission::Goal::ConstSharedPtr goal) {
        return handleGoal(*goal);
      },
      [this](const std::shared_ptr<GoalHandle> & goal) { return handleCancel(goal); },
      [this](const std::shared_ptr<GoalHandle> & goal) { handleAccepted(goal); });
    control_ = node_->create_service<ControlMission>(
      "control_mission", [this](
                           const std::shared_ptr<ControlMission::Request> request,
                           std::shared_ptr<ControlMission::Response> response) {
        handleControl(*request, *response);
      });

    mission_.onReadiness([this](bool ready, const std::vector<std::string> & errors) {
      ready_ = ready;
      readiness_errors_ = errors;
      if (ready) {
        startActivationIfLoading();
      }
    });
    mission_.onActivated([this]() { onActivated(); });
    mission_.onDeactivated([this]() { onDeactivated(); });
    mission_.onProgress([this](int index) { onProgress(index); });
    mission_.onCompleted([this]() { onCompleted(); });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() { tick(); });

    publishIdle();
  }

private:
  enum class Phase
  {
    Loading,
    Activating,
    Running,
    Suspending,
    Paused,
    Resuming,
    Aborting,
    Completing
  };

  struct Active
  {
    std::shared_ptr<GoalHandle> goal;
    fleet_interfaces::msg::MissionPlan plan;
    std::vector<uint8_t> status;
    std::vector<int> first_item;
    std::vector<int> last_item;
    std::vector<bool> owns_item;
    bool ends_landed{false};
    Phase phase{Phase::Loading};
    std::string abort_reason;
    rclcpp::Time deadline;
    std::optional<rclcpp::Time> arm_at;
  };

  rclcpp_action::GoalResponse handleGoal(const ExecuteMission::Goal & goal)
  {
    const auto & plan = goal.plan;
    if (active_) {
      RCLCPP_ERROR(
        node_->get_logger(), "accept mission %d failed: cause: mission %d is active", plan.id,
        active_->plan.id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (mode_active_) {
      RCLCPP_ERROR(
        node_->get_logger(), "accept mission %d failed: cause: mission mode still active", plan.id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (plan.commands.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "accept mission %d failed: cause: no commands", plan.id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & command : plan.commands) {
      if (!supported(command.kind)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "accept mission %d failed: cause: command %d has unsupported kind %u", plan.id,
          command.id, command.kind);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    if (!reference_altitude_) {
      RCLCPP_ERROR(
        node_->get_logger(), "accept mission %d failed: cause: not ready: %s", plan.id,
        kNoReferenceAltitude);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> & goal)
  {
    if (!active_ || active_->goal != goal) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> & goal)
  {
    auto active = std::make_unique<Active>();
    active->goal = goal;
    active->plan = goal->get_goal()->plan;
    active->status.assign(active->plan.commands.size(), MissionState::COMMAND_WAITING);
    active->deadline = node_->now() + rclcpp::Duration::from_seconds(kReadyTimeoutS);

    px4_ros2::Mission mission;
    try {
      mission = buildMission(*active);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "build mission %d failed: cause: %s", active->plan.id, error.what());
      active_ = std::move(active);
      finishFailed(
        active_command_id(), std::string("build mission failed: cause: ") + error.what());
      return;
    }
    active_ = std::move(active);
    RCLCPP_INFO(
      node_->get_logger(), "mission %d accepted with %zu commands", active_->plan.id,
      active_->plan.commands.size());
    plan_pub_->publish(active_->plan);
    publishState(MissionState::MISSION_RUNNING);
    ready_ = false;
    readiness_errors_.clear();
    mission_.setMission(mission);
    startActivationIfLoading();
  }

  void handleControl(const ControlMission::Request & request, ControlMission::Response & response)
  {
    if (!active_) {
      response.accepted = false;
      response.reason = "no active mission";
      return;
    }
    if (request.request == ControlMission::Request::SUSPEND) {
      if (active_->phase != Phase::Running) {
        response.accepted = false;
        response.reason = "mission is not running";
        return;
      }
      active_->phase = Phase::Suspending;
      active_->deadline = node_->now() + rclcpp::Duration::from_seconds(kDeactivationTimeoutS);
      sendNavState(VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
      response.accepted = true;
      return;
    }
    if (request.request == ControlMission::Request::RESUME) {
      if (active_->phase != Phase::Paused) {
        response.accepted = false;
        response.reason = "mission is not suspended";
        return;
      }
      active_->phase = Phase::Resuming;
      active_->deadline = node_->now() + rclcpp::Duration::from_seconds(kActivationTimeoutS);
      sendNavState(mode_id_);
      response.accepted = true;
      return;
    }
    response.accepted = false;
    response.reason = "unknown request " + std::to_string(request.request);
  }

  std::optional<std::string> activationBlocker() const
  {
    if (!ready_) {
      return readiness_errors_.empty() ? "executor has not reported the mission ready"
                                       : "executor not ready: " + joinErrors();
    }
    if (!status_) {
      return "no vehicle status from PX4";
    }
    if (!status_->pre_flight_checks_pass) {
      return "PX4 pre-flight checks fail in navigation state " + std::to_string(status_->nav_state);
    }
    return std::nullopt;
  }

  void startActivationIfLoading()
  {
    if (!active_ || active_->phase != Phase::Loading || activationBlocker()) {
      return;
    }
    active_->phase = Phase::Activating;
    active_->deadline = node_->now() + rclcpp::Duration::from_seconds(kActivationTimeoutS);
    sendNavState(mode_id_);
    active_->arm_at = node_->now() + rclcpp::Duration::from_seconds(kArmDelayS);
  }

  void onActivated()
  {
    mode_active_ = true;
    if (!active_) {
      return;
    }
    if (active_->phase == Phase::Activating || active_->phase == Phase::Resuming) {
      active_->phase = Phase::Running;
      RCLCPP_INFO(node_->get_logger(), "mission %d running", active_->plan.id);
      publishState(MissionState::MISSION_RUNNING);
    }
  }

  void onProgress(int index)
  {
    if (!active_ || active_->phase != Phase::Running) {
      return;
    }
    for (size_t i = 0; i < active_->plan.commands.size(); ++i) {
      const int first = active_->first_item[i];
      const int last = active_->last_item[i];
      uint8_t status = MissionState::COMMAND_WAITING;
      if (last < index) {
        status = MissionState::COMMAND_FINISHED;
      } else if (active_->owns_item[i] && first <= index && index <= last) {
        status = MissionState::COMMAND_RUNNING;
      }
      setStatus(i, status);
    }
    publishState(MissionState::MISSION_RUNNING);
  }

  void onCompleted()
  {
    if (!active_ || active_->phase == Phase::Aborting) {
      return;
    }
    if (!mode_active_) {
      finishSucceeded();
      return;
    }
    active_->phase = Phase::Completing;
    active_->deadline = node_->now() + rclcpp::Duration::from_seconds(kDeactivationTimeoutS);
    if (!active_->ends_landed) {
      sendNavState(VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
    }
  }

  void finishSucceeded()
  {
    for (size_t i = 0; i < active_->plan.commands.size(); ++i) {
      setStatus(i, MissionState::COMMAND_FINISHED);
    }
    publishState(MissionState::MISSION_FINISHED);
    auto result = std::make_shared<ExecuteMission::Result>();
    result->stamp = node_->now();
    result->status = MissionState::MISSION_FINISHED;
    result->command_id = active_->plan.commands.back().id;
    active_->goal->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "mission %d finished", active_->plan.id);
    active_.reset();
  }

  void onDeactivated()
  {
    mode_active_ = false;
    if (!active_) {
      return;
    }
    switch (active_->phase) {
      case Phase::Suspending: {
        active_->phase = Phase::Paused;
        const int current = currentIndex();
        if (current >= 0) {
          setStatus(current, MissionState::COMMAND_PAUSED);
        }
        publishState(MissionState::MISSION_PAUSED);
        return;
      }
      case Phase::Aborting:
        finishAborted();
        return;
      case Phase::Completing:
        finishSucceeded();
        return;
      case Phase::Loading:
      case Phase::Activating:
      case Phase::Running:
      case Phase::Resuming:
        finishFailed(active_command_id(), "mission mode deactivated by the flight controller");
        return;
      case Phase::Paused:
        return;
    }
  }

  void tick()
  {
    if (!active_) {
      return;
    }
    if (active_->goal->is_canceling()) {
      requestAbort("cancelled by the manager");
      if (!active_) {
        return;
      }
    }
    const auto now = node_->now();
    if (active_->arm_at && now >= *active_->arm_at) {
      active_->arm_at.reset();
      sendArm();
    }
    switch (active_->phase) {
      case Phase::Loading:
        startActivationIfLoading();
        if (active_->phase == Phase::Loading && now > active_->deadline) {
          finishFailed(active_command_id(), activationBlocker().value());
        }
        break;
      case Phase::Activating:
      case Phase::Resuming:
        if (now > active_->deadline) {
          finishFailed(active_command_id(), "mission mode did not activate");
        }
        break;
      case Phase::Suspending:
        if (now > active_->deadline) {
          finishFailed(active_command_id(), "loiter did not end the mission mode");
        }
        break;
      case Phase::Aborting:
        if (now > active_->deadline) {
          finishFailed(active_command_id(), "return to launch did not end the mission mode");
        }
        break;
      case Phase::Completing:
        if (now > active_->deadline) {
          finishFailed(active_command_id(), "the mission mode did not end after the mission");
        }
        break;
      default:
        break;
    }
  }

  void requestAbort(const std::string & reason)
  {
    if (!active_ || active_->phase == Phase::Aborting || active_->phase == Phase::Completing) {
      return;
    }
    const auto previous = active_->phase;
    active_->phase = Phase::Aborting;
    active_->abort_reason = reason;
    active_->arm_at.reset();
    active_->deadline = node_->now() + rclcpp::Duration::from_seconds(kDeactivationTimeoutS);
    if (previous != Phase::Loading) {
      releaseMissionMode();
    }
    if (!mode_active_) {
      finishAborted();
    }
  }

  void finishAborted()
  {
    const int current = currentIndex();
    const int command_id = current >= 0 ? active_->plan.commands[current].id : -1;
    if (current >= 0) {
      setStatus(current, MissionState::COMMAND_ABORTED);
    }
    publishState(MissionState::MISSION_ABORTED, command_id);
    auto result = std::make_shared<ExecuteMission::Result>();
    result->stamp = node_->now();
    result->status = MissionState::MISSION_ABORTED;
    result->command_id = command_id;
    result->reason = active_->abort_reason;
    if (active_->goal->is_canceling()) {
      active_->goal->canceled(result);
    } else {
      active_->goal->abort(result);
    }
    RCLCPP_WARN(
      node_->get_logger(), "mission %d aborted: %s", active_->plan.id,
      active_->abort_reason.c_str());
    active_.reset();
  }

  void finishFailed(int command_id, const std::string & reason)
  {
    const int current = currentIndex();
    if (current >= 0) {
      setStatus(current, MissionState::COMMAND_FAILED);
    }
    publishState(MissionState::MISSION_FAILED, command_id);
    auto result = std::make_shared<ExecuteMission::Result>();
    result->stamp = node_->now();
    result->status = MissionState::MISSION_FAILED;
    result->command_id = command_id;
    result->reason = reason;
    active_->goal->abort(result);
    RCLCPP_ERROR(
      node_->get_logger(), "mission %d failed: cause: %s", active_->plan.id, reason.c_str());
    active_.reset();
  }

  static bool supported(uint8_t kind)
  {
    switch (kind) {
      case MissionCommand::TAKEOFF:
      case MissionCommand::LAND:
      case MissionCommand::WAYPOINT:
      case MissionCommand::HOME:
        return true;
      default:
        return false;
    }
  }

  px4_ros2::Mission buildMission(Active & active)
  {
    const double reference = *reference_altitude_;
    nlohmann::json items = nlohmann::json::array();
    const size_t count = active.plan.commands.size();
    active.first_item.assign(count, -1);
    active.last_item.assign(count, -1);
    active.owns_item.assign(count, false);
    int last_item = -1;
    double altitude_agl = 0.0;
    for (size_t i = 0; i < count; ++i) {
      const auto & command = active.plan.commands[i];
      const std::string id = std::to_string(command.id);
      const int first = static_cast<int>(items.size());
      switch (command.kind) {
        case MissionCommand::TAKEOFF:
          altitude_agl = command.altitude_agl;
          items.push_back(
            {{"type", "takeoff"}, {"id", id}, {"altitude", reference + altitude_agl}});
          break;
        case MissionCommand::WAYPOINT:
        case MissionCommand::HOME: {
          if (command.altitude_agl > 0.0) {
            altitude_agl = command.altitude_agl;
          }
          const bool home = command.kind == MissionCommand::HOME;
          items.push_back(
            {{"type", "navigation"},
             {"navigationType", "waypoint"},
             {"id", id},
             {"frame", "global"},
             {"x", home ? active.plan.home.latitude : command.latitude},
             {"y", home ? active.plan.home.longitude : command.longitude},
             {"z", reference + altitude_agl}});
          if (command.hold_s > 0.0) {
            items.push_back({{"type", "hold"}, {"duration", command.hold_s}});
          }
          break;
        }
        case MissionCommand::LAND:
          items.push_back({{"type", "land"}, {"id", id}});
          break;
        default:
          break;
      }
      if (static_cast<int>(items.size()) > first) {
        active.owns_item[i] = true;
        active.first_item[i] = first;
        active.last_item[i] = static_cast<int>(items.size()) - 1;
        last_item = active.last_item[i];
      } else {
        active.first_item[i] = last_item;
        active.last_item[i] = last_item;
      }
    }
    if (items.empty()) {
      throw std::runtime_error("the plan holds no position command");
    }
    active.ends_landed = items.back()["type"] == "land";
    nlohmann::json document;
    document["version"] = 1;
    document["mission"]["defaults"]["horizontalVelocity"] = horizontal_velocity_;
    document["mission"]["defaults"]["verticalVelocity"] = vertical_velocity_;
    document["mission"]["defaults"]["maxHeadingRate"] = max_heading_rate_;
    document["mission"]["items"] = items;
    return document.get<px4_ros2::Mission>();
  }

  void releaseMissionMode()
  {
    if (status_.value().arming_state != VehicleStatus::ARMING_STATE_ARMED) {
      sendNavState(VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
      return;
    }
    VehicleCommand command;
    command.command = VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    send(command);
  }

  void sendNavState(uint8_t nav_state)
  {
    VehicleCommand command;
    command.command = VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
    command.param1 = static_cast<float>(nav_state);
    send(command);
  }

  void sendArm()
  {
    VehicleCommand command;
    command.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    command.param1 = 1.0F;
    command.param2 = force_arm_ ? 21196.0F : 0.0F;
    send(command);
  }

  void send(VehicleCommand command)
  {
    command.timestamp = static_cast<uint64_t>(node_->now().nanoseconds() / 1000);
    const auto & status = status_.value();
    command.target_system = status.system_id;
    command.target_component = status.component_id;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    command_pub_->publish(command);
  }

  int currentIndex() const
  {
    if (!active_) {
      return -1;
    }
    for (size_t i = 0; i < active_->status.size(); ++i) {
      const auto status = active_->status[i];
      if (status == MissionState::COMMAND_RUNNING || status == MissionState::COMMAND_PAUSED) {
        return static_cast<int>(i);
      }
    }
    for (size_t i = 0; i < active_->status.size(); ++i) {
      if (active_->status[i] == MissionState::COMMAND_WAITING) {
        return static_cast<int>(i);
      }
    }
    return static_cast<int>(active_->status.size()) - 1;
  }

  int active_command_id() const
  {
    const int current = currentIndex();
    return current >= 0 ? active_->plan.commands[current].id : -1;
  }

  void setStatus(size_t index, uint8_t status)
  {
    if (active_->status[index] == status) {
      return;
    }
    active_->status[index] = status;
    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->stamp = node_->now();
    feedback->command_id = active_->plan.commands[index].id;
    feedback->status = status;
    active_->goal->publish_feedback(feedback);
  }

  void publishState(uint8_t mission_status, std::optional<int> current_command_id = std::nullopt)
  {
    MissionState msg;
    msg.mission_id = active_->plan.id;
    msg.current_command_id = current_command_id.value_or(active_command_id());
    for (const auto & command : active_->plan.commands) {
      msg.command_ids.push_back(command.id);
    }
    msg.command_status = active_->status;
    msg.mission_status = mission_status;
    publish(msg);
  }

  void publishIdle()
  {
    MissionState msg;
    msg.mission_id = -1;
    msg.current_command_id = -1;
    msg.mission_status = MissionState::MISSION_IDLE;
    publish(msg);
  }

  void publish(MissionState & msg)
  {
    msg.stamp = node_->now();
    msg.ready = reference_altitude_.has_value();
    msg.not_ready_reason = msg.ready ? "" : kNoReferenceAltitude;
    state_pub_->publish(msg);
  }

  std::string joinErrors() const
  {
    std::string joined;
    for (const auto & error : readiness_errors_) {
      if (!joined.empty()) {
        joined += "; ";
      }
      joined += error;
    }
    return joined;
  }

  std::shared_ptr<rclcpp::Node> node_;
  AeroscoutMission & mission_;
  uint8_t mode_id_;
  bool mode_active_{false};
  double horizontal_velocity_{0.0};
  double vertical_velocity_{0.0};
  double max_heading_rate_{0.0};
  bool force_arm_{false};
  rclcpp::Publisher<fleet_interfaces::msg::MissionPlan>::SharedPtr plan_pub_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr server_;
  rclcpp::Service<ControlMission>::SharedPtr control_;
  rclcpp::Publisher<MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<VehicleStatus> status_;
  std::optional<double> reference_altitude_;
  bool ready_{false};
  std::vector<std::string> readiness_errors_;
  std::unique_ptr<Active> active_;
};

}  // namespace aeroscout_mission
