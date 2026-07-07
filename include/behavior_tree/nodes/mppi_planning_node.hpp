#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/mppi_planning.hpp"

class MppiPlanningNode : public BT::RosActionNode<system_interfaces::action::MPPIPlanning>
{
public:
  using ActionT = system_interfaces::action::MPPIPlanning;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  MppiPlanningNode(const std::string& name, const BT::NodeConfig& config,
                   const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("target_lat", 0.0, "Goal latitude"),
      BT::InputPort<double>("target_lon", 0.0, "Goal longitude")
    });
  }

  void on_tick(Goal& goal)
  {
    getInput("target_lat", goal.goal_lat);
    getInput("target_lon", goal.goal_lon);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "MPPI: dist=%.2f obstacle=%s",
                 feedback->distance_to_goal_m,
                 feedback->obstacle_present ? "true" : "false");
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
