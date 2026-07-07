#pragma once

#include <cmath>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class WaypointReached : public BT::ConditionNode
{
public:
  WaypointReached(const std::string& name, const BT::NodeConfig& config,
                  const BT::RosNodeParams& params)
    : BT::ConditionNode(name, config), node_(params.nh)
  {
    if (!node_) {
      throw BT::RuntimeError("WaypointReached requires a ROS node");
    }

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        got_odom_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("target_lat", 0.0, "Target latitude"),
      BT::InputPort<double>("target_lon", 0.0, "Target longitude"),
      BT::InputPort<double>("tolerance", 1.5, "Arrival tolerance")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!got_odom_) {
      return BT::NodeStatus::FAILURE;
    }

    double target_lat = 0.0;
    double target_lon = 0.0;
    double tolerance = 1.5;
    getInput("target_lat", target_lat);
    getInput("target_lon", target_lon);
    getInput("tolerance", tolerance);

    const double dx = target_lat - current_x_;
    const double dy = target_lon - current_y_;
    return std::hypot(dx, dy) <= tolerance ? BT::NodeStatus::SUCCESS
                                           : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  bool got_odom_ = false;
};
