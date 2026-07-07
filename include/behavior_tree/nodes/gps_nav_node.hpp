#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/gps_nav.hpp"

class GpsNavNode : public BT::RosActionNode<system_interfaces::action::GPSNav>
{
public:
  using ActionT = system_interfaces::action::GPSNav;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  GpsNavNode(const std::string& name, const BT::NodeConfig& config,
             const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("lat", "Target latitude"),
      BT::InputPort<double>("lon", "Target longitude"),
      BT::InputPort<double>("tolerance", 1.5, "Arrival tolerance")
    });
  }

  void on_tick(Goal& goal)
  {
    if (!getInput("lat", goal.latitude)) {
      throw BT::RuntimeError("GpsNavNode missing input [lat]");
    }
    if (!getInput("lon", goal.longitude)) {
      throw BT::RuntimeError("GpsNavNode missing input [lon]");
    }
    getInput("tolerance", goal.tolerance);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "GPSNav: remaining=%.2f m", feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus on_success(const WrappedResult& result)
  {
    return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  bool setGoal(Goal& goal) override
  {
    on_tick(goal);
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    return on_feedback(feedback);
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    return on_success(result);
  }
};
