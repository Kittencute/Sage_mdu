/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/mission/actions/action.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace aeroscout_mission
{

class HoverAction : public px4_ros2::ActionInterface
{
public:
  explicit HoverAction(px4_ros2::ModeBase & mode)
  : node_(mode.node()), global_position_(std::make_shared<px4_ros2::OdometryGlobalPosition>(mode))
  {
  }

  ~HoverAction() override = default;

  std::string name() const override { return "hold"; }

  bool shouldStopAtWaypoint(const px4_ros2::ActionArguments &) override { return true; }

  bool canRun(
    const px4_ros2::ActionArguments & arguments, std::vector<std::string> & errors) override
  {
    if (arguments.contains("duration") && arguments.at<float>("duration") <= 0.0F) {
      errors.push_back("hold duration must be positive");
      return false;
    }
    if (!global_position_->positionValid()) {
      errors.push_back("global position not valid");
      return false;
    }
    return true;
  }

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    float duration_s = 1.0F;
    if (arguments.contains("duration")) {
      duration_s = arguments.at<float>("duration");
    }
    const std::vector<px4_ros2::MissionItem> hold_items{
      px4_ros2::NavigationItem(px4_ros2::Waypoint(global_position_->position()))};
    handler->runTrajectory(std::make_shared<px4_ros2::Mission>(hold_items), [] {}, true);
    done_timer_ = rclcpp::create_timer(
      &node_, node_.get_clock(), rclcpp::Duration::from_seconds(duration_s), [this, on_completed] {
        done_timer_.reset();
        on_completed();
      });
  }

  void deactivate() override { done_timer_.reset(); }

private:
  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> global_position_;
  rclcpp::TimerBase::SharedPtr done_timer_;
};

}  // namespace aeroscout_mission
