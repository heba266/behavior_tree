#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/stop_rover.hpp"

class StopRoverNode : public BT::RosActionNode<system_interfaces::action::StopRover>
{
public:
  using ActionT = system_interfaces::action::StopRover;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  StopRoverNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("duration_sec", 1.0, "Stop duration")
    });
  }

  void on_tick(Goal& goal)
  {
    getInput("duration_sec", goal.duration_sec);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "StopRover: elapsed=%.1f", feedback->elapsed_sec);
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
