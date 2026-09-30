/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/utils/geodesic.hpp>
#include <rclcpp/rclcpp.hpp>

namespace aeroscout_mission
{

class DirectWaypointTrajectoryExecutor : public px4_ros2::TrajectoryExecutorInterface
{
public:
  static constexpr float kApproachGain = 1.0F;

  explicit DirectWaypointTrajectoryExecutor(
    px4_ros2::ModeBase & mode, float acceptance_radius = 2.0f)
  : acceptance_radius_(acceptance_radius), node_(mode.node())
  {
    setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(mode);
    map_projection_ = std::make_unique<px4_ros2::MapProjection>(mode);
    vehicle_global_position_ = std::make_shared<px4_ros2::OdometryGlobalPosition>(mode);
    vehicle_attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(mode);
  }

  ~DirectWaypointTrajectoryExecutor() override = default;

  bool navigationItemTypeSupported(px4_ros2::NavigationItemType type) override
  {
    return type == px4_ros2::NavigationItemType::Waypoint;
  }

  bool frameSupported(px4_ros2::MissionFrame frame) override
  {
    return frame == px4_ros2::MissionFrame::Global;
  }

  void runTrajectory(const TrajectoryConfig & config) override
  {
    current_trajectory_ = config;
    current_index_ = config.start_index;
    holding_ = false;
    horizontal_speed_limit_ = config.options.horizontal_velocity.value();
    vertical_speed_limit_ = config.options.vertical_velocity.value();
  }

  void updateSetpoint() override
  {
    if (!current_index_) {
      return;
    }

    const auto * navigation_item = std::get_if<px4_ros2::NavigationItem>(
      &current_trajectory_.trajectory->items()[*current_index_]);
    if (!navigation_item) {
      continueNextItem();
      return;
    }

    if (!vehicle_global_position_->positionValid()) {
      RCLCPP_ERROR(node_.get_logger(), "run trajectory failed: cause: global position not valid");
      current_trajectory_.on_failure();
      return;
    }

    if (!map_projection_->isInitialized()) {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "update trajectory setpoint failed: cause: map projection not initialized");
      return;
    }

    const auto & waypoint = std::get<px4_ros2::Waypoint>(navigation_item->data);
    const Eigen::Vector3d & target_global = waypoint.coordinate;

    const Eigen::Vector3f target_local = map_projection_->globalToLocal(target_global);

    std::optional<float> heading_rad;
    if (
      px4_ros2::horizontalDistanceToGlobalPosition(
        vehicle_global_position_->position(), target_global) > 0.1f) {
      heading_rad =
        px4_ros2::headingToGlobalPosition(vehicle_global_position_->position(), target_global);
    }

    if (holding_) {
      setpoint_->updatePosition(target_local);
      return;
    }
    const Eigen::Vector3f position_local =
      map_projection_->globalToLocal(vehicle_global_position_->position());
    setpoint_->update(limitedVelocity(target_local - position_local), std::nullopt, heading_rad);

    float radius = acceptance_radius_;
    if (*current_index_ == current_trajectory_.end_index && current_trajectory_.stop_at_last) {
      radius /= 2.f;
    }

    if (positionReached(target_global, radius)) {
      continueNextItem();
    }
  }

private:
  Eigen::Vector3f limitedVelocity(const Eigen::Vector3f & error_ned) const
  {
    const Eigen::Vector3f velocity = kApproachGain * error_ned;
    const float horizontal = velocity.head<2>().norm();
    const float vertical = std::abs(velocity.z());
    const float scale =
      std::min({1.0F, horizontal_speed_limit_ / horizontal, vertical_speed_limit_ / vertical});
    return velocity * scale;
  }

  void continueNextItem()
  {
    const int index_reached = *current_index_;
    if (index_reached == current_trajectory_.end_index && current_trajectory_.stop_at_last) {
      holding_ = true;
    } else {
      current_index_ = index_reached + 1;
      if (*current_index_ > current_trajectory_.end_index) {
        current_index_.reset();
      }
    }
    current_trajectory_.on_index_reached(index_reached);
  }

  bool positionReached(const Eigen::Vector3d & target, float radius) const
  {
    return px4_ros2::distanceToGlobalPosition(vehicle_global_position_->position(), target) <
           radius;
  }

  TrajectoryConfig current_trajectory_;
  const float acceptance_radius_;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> setpoint_;
  std::unique_ptr<px4_ros2::MapProjection> map_projection_;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> vehicle_global_position_;
  std::shared_ptr<px4_ros2::OdometryAttitude> vehicle_attitude_;
  std::optional<int> current_index_;
  bool holding_{false};
  float horizontal_speed_limit_{0.0F};
  float vertical_speed_limit_{0.0F};
  rclcpp::Node & node_;
};

}  // namespace aeroscout_mission
