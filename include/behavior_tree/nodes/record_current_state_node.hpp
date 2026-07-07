#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

class RecordCurrentStateNode : public BT::SyncActionNode
{
public:
  RecordCurrentStateNode(const std::string& name, const BT::NodeConfig& config,
                         const BT::RosNodeParams& params)
    : BT::SyncActionNode(name, config), node_(params.nh) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "Current state recorded by BT placeholder");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};
