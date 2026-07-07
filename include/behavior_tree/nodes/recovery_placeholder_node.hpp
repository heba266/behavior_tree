#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

class RecoveryPlaceholderNode : public BT::SyncActionNode
{
public:
  RecoveryPlaceholderNode(const std::string& name, const BT::NodeConfig& config,
                          const BT::RosNodeParams& params)
    : BT::SyncActionNode(name, config), node_(params.nh) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "%s has no action interface yet", name().c_str());
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
};
