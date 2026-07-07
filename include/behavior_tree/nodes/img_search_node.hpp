#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/action/img_search.hpp"

class ImgSearchNode : public BT::RosActionNode<system_interfaces::action::ImgSearch>
{
public:
  using ActionT = system_interfaces::action::ImgSearch;
  using Base = BT::RosActionNode<ActionT>;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;
  using WrappedResult = typename Base::WrappedResult;

  ImgSearchNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params)
    : Base(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("timeout_sec", 60.0, "Image search timeout"),
      BT::OutputPort<bool>("img_found", "True when target image is found")
    });
  }

  void on_tick(Goal& goal)
  {
    getInput("timeout_sec", goal.timeout_sec);
  }

  BT::NodeStatus on_feedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_DEBUG(logger(), "ImgSearch: elapsed=%.1f checked=%d",
                 feedback->elapsed_sec, feedback->faces_checked);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus on_success(const WrappedResult& result)
  {
    setOutput("img_found", result.result->image_found);
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
