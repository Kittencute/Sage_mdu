#pragma once

#include <aeroscout_mission/direct_waypoint_trajectory_executor.hpp>
#include <aeroscout_mission/hover_action.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace aeroscout_mission
{

class MissionExecutorWithModeId : public px4_ros2::MissionExecutor
{
public:
  using px4_ros2::MissionExecutor::MissionExecutor;
  using px4_ros2::MissionExecutor::modeId;
};

class AeroscoutMission
{
public:
  static constexpr const char * kModeName = "Aeroscout Mission";

  explicit AeroscoutMission(const std::shared_ptr<rclcpp::Node> & node) : node_(node)
  {
    auto config = px4_ros2::MissionExecutor::Configuration()
                    .withTrajectoryExecutor<DirectWaypointTrajectoryExecutor>()
                    .addCustomAction<HoverAction>();

    mission_executor_ = std::make_unique<MissionExecutorWithModeId>(kModeName, config, *node_);

    if (!mission_executor_->doRegister()) {
      throw std::runtime_error("register mission executor with PX4 failed");
    }

    progress_pub_ =
      node_->create_publisher<std_msgs::msg::String>("mission_progress", rclcpp::QoS(10));

    mission_executor_->onProgressUpdate([this](int current_index) {
      RCLCPP_INFO(node_->get_logger(), "Mission progress: index=%d", current_index);
      publishProgress(current_index, "active");
      if (on_progress_) {
        on_progress_(current_index);
      }
    });

    mission_executor_->onCompleted([this]() {
      RCLCPP_INFO(node_->get_logger(), "Mission completed!");
      publishProgress(static_cast<int>(mission_executor_->mission().items().size()), "completed");
      if (on_completed_) {
        on_completed_();
      }
    });

    mission_executor_->onReadynessUpdate(
      [this](bool ready, const std::vector<std::string> & errors) {
        logReadinessUpdate(ready, errors);
        if (on_readiness_) {
          on_readiness_(ready, errors);
        }
      });

    mission_executor_->onActivated([this]() {
      RCLCPP_INFO(node_->get_logger(), "Aeroscout Mission activated");
      if (on_activated_) {
        on_activated_();
      }
    });

    mission_executor_->onDeactivated([this]() {
      RCLCPP_INFO(node_->get_logger(), "Aeroscout Mission deactivated");
      if (on_deactivated_) {
        on_deactivated_();
      }
    });

    RCLCPP_INFO(node_->get_logger(), "Aeroscout Mission registered");
  }

  void setMission(const px4_ros2::Mission & mission) { mission_executor_->setMission(mission); }
  px4_ros2::ModeBase::ModeID modeId() const { return mission_executor_->modeId(); }
  void onProgress(std::function<void(int)> callback) { on_progress_ = std::move(callback); }
  void onCompleted(std::function<void()> callback) { on_completed_ = std::move(callback); }
  void onActivated(std::function<void()> callback) { on_activated_ = std::move(callback); }
  void onDeactivated(std::function<void()> callback) { on_deactivated_ = std::move(callback); }
  void onReadiness(std::function<void(bool, const std::vector<std::string> &)> callback)
  {
    on_readiness_ = std::move(callback);
  }

private:
  void logReadinessUpdate(bool ready, const std::vector<std::string> & errors)
  {
    if (
      last_ready_state_.has_value() && *last_ready_state_ == ready &&
      (ready || errors == last_readiness_errors_)) {
      return;
    }

    last_ready_state_ = ready;
    last_readiness_errors_ = errors;

    if (ready) {
      RCLCPP_INFO(node_->get_logger(), "Mission ready");
      return;
    }

    if (errors.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Mission not ready");
      return;
    }

    std::string joined_errors;
    for (size_t i = 0; i < errors.size(); ++i) {
      if (i != 0) {
        joined_errors += "; ";
      }
      joined_errors += errors[i];
    }

    RCLCPP_WARN(node_->get_logger(), "Mission not ready: %s", joined_errors.c_str());
  }

  void publishProgress(int item_index, const std::string & state)
  {
    std_msgs::msg::String msg;
    const auto & items = mission_executor_->mission().items();
    int waypoint_index = 0;
    int total_waypoints = 0;
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
      const auto & item = items[i];
      if (std::holds_alternative<px4_ros2::NavigationItem>(item)) {
        if (i < item_index) {
          ++waypoint_index;
        }
        ++total_waypoints;
      }
    }
    msg.data = "{\"index\":" + std::to_string(waypoint_index) +
               ",\"total\":" + std::to_string(total_waypoints) + ",\"state\":\"" + state + "\"}";
    progress_pub_->publish(msg);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<MissionExecutorWithModeId> mission_executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progress_pub_;
  std::optional<bool> last_ready_state_;
  std::vector<std::string> last_readiness_errors_;
  std::function<void(int)> on_progress_;
  std::function<void()> on_completed_;
  std::function<void()> on_activated_;
  std::function<void()> on_deactivated_;
  std::function<void(bool, const std::vector<std::string> &)> on_readiness_;
};

}  // namespace aeroscout_mission
