#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/lane_follow.hpp"

class LaneFollowNode : public BT::RosActionNode<system_interfaces::action::LaneFollow>
{
public:
  using ActionT = system_interfaces::action::LaneFollow;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  LaneFollowNode(const std::string& name, const BT::NodeConfig& config,
                 const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<std::string>("phase_id", "1", "Lane mission phase"),
      BT::OutputPort<std::string>("exit_reason", "Action completion reason")
    });
  }

  void on_tick(Goal& goal)
  {
    getInput("phase_id", goal.phase_id);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "LaneFollow: v=%.2f w=%.2f obstacle=%s",
                 feedback->linear_speed, feedback->angular_correction,
                 feedback->obstacle_detected ? "true" : "false");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus on_success(const WrappedResult& result)
  {
    setOutput("exit_reason", result.result->reason);
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
