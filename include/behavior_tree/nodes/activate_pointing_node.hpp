#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/activate_pointing.hpp"

class ActivatePointingNode : public BT::RosActionNode<system_interfaces::action::ActivatePointing>
{
public:
  using ActionT = system_interfaces::action::ActivatePointing;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  ActivatePointingNode(const std::string& name, const BT::NodeConfig& config,
                       const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("duration_sec", 3.0, "Pointing duration")
    });
  }

  void on_tick(Goal& goal)
  {
    getInput("duration_sec", goal.duration_sec);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "ActivatePointing: elapsed=%.1f", feedback->elapsed_sec);
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
