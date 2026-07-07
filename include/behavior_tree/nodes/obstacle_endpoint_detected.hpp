#pragma once

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ObstacleEndPointDetected : public BT::ConditionNode
{
public:
  ObstacleEndPointDetected(const std::string& name, const BT::NodeConfig& config,
                           const BT::RosNodeParams& params)
    : BT::ConditionNode(name, config), node_(params.nh)
  {
    if (!node_) {
      throw BT::RuntimeError("ObstacleEndPointDetected requires a ROS node");
    }

    obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/obstacle_flag", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { obstacle_detected_ = msg->data; });

    endpoint_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/finish_line_detected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { endpoint_detected_ = msg->data; });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("phase_id", "1", "Lane phase: 1 or 6")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string phase_id = "1";
    getInput("phase_id", phase_id);

    if (phase_id == "1") {
      return obstacle_detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    if (phase_id == "6") {
      return endpoint_detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Unknown lane phase_id '%s'", phase_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr endpoint_sub_;
  bool obstacle_detected_ = false;
  bool endpoint_detected_ = false;
};
