#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behavior_tree/nodes/activate_pointing_node.hpp"
#include "behavior_tree/nodes/gps_nav_node.hpp"
#include "behavior_tree/nodes/img_search_node.hpp"
#include "behavior_tree/nodes/lane_follow_node.hpp"
#include "behavior_tree/nodes/mppi_planning_node.hpp"
#include "behavior_tree/nodes/obstacle_endpoint_detected.hpp"
#include "behavior_tree/nodes/record_current_state_node.hpp"
#include "behavior_tree/nodes/recovery_placeholder_node.hpp"
#include "behavior_tree/nodes/set_autonomous_light_node.hpp"
#include "behavior_tree/nodes/stop_rover_node.hpp"
#include "behavior_tree/nodes/waypoint_reached.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

class BTExecutorNode : public rclcpp::Node
{
public:
  BTExecutorNode() : rclcpp::Node("bt_executor_node")
  {
    declare_parameter("tree_xml_path", "/home/heba/ros2_ws/src/behavior_tree/trees/bt3.xml");
    declare_parameter("tick_rate_ms", 100);
    declare_parameter("wp1_lat", 30.0444);
    declare_parameter("wp1_lon", 31.2357);
    declare_parameter("wp2_lat", 30.0450);
    declare_parameter("wp2_lon", 31.2360);
    declare_parameter("wp3_lat", 30.0448);
    declare_parameter("wp3_lon", 31.2350);
    declare_parameter("end_point_lat", 30.0444);
    declare_parameter("end_point_lon", 31.2357);
    declare_parameter("given_img", "");
  }

  void run()
  {
    BT::RosNodeParams params;
    params.nh = shared_from_this();
    params.server_timeout = std::chrono::milliseconds(5000);
    params.wait_for_server_timeout = std::chrono::milliseconds(5000);

    BT::BehaviorTreeFactory factory;
    registerNodes(factory, params);

    const auto xml_path = get_parameter("tree_xml_path").as_string();
    BT::Tree tree;
    try {
      tree = factory.createTreeFromFile(xml_path);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load BT XML '%s': %s", xml_path.c_str(), ex.what());
      return;
    }

    setBlackboardValues(tree);

    BT::StdCoutLogger cout_logger(tree);
    BT::FileLogger2 file_logger(tree, "/tmp/ugvc_bt_run.btlog");

    const auto tick_rate_ms = get_parameter("tick_rate_ms").as_int();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      status = tree.tickOnce();
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(tick_rate_ms));
    }

    tree.haltTree();
    const auto status_text = std::string(BT::toStr(status, true));
    RCLCPP_INFO(get_logger(), "BT finished with status %s", status_text.c_str());
  }

private:
  template <typename NodeT>
  void registerAction(BT::BehaviorTreeFactory& factory, const std::string& bt_id,
                      const std::string& action_name, const BT::RosNodeParams& base_params)
  {
    BT::RosNodeParams params = base_params;
    params.default_port_value = action_name;
    factory.registerNodeType<NodeT>(bt_id, params);
  }

  void registerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
  {
    registerAction<LaneFollowNode>(factory, "lane_follow_action", "lane_follow_action", params);
    registerAction<MppiPlanningNode>(factory, "mppi_planning_action", "mppi_planning_action", params);
    registerAction<GpsNavNode>(factory, "gps_nav_action", "gps_nav_action", params);
    registerAction<ImgSearchNode>(factory, "img_search_action", "img_search_action", params);
    registerAction<StopRoverNode>(factory, "stop_rover", "stop_rover", params);
    registerAction<SetAutonomousLightNode>(
      factory, "set_autonomous_light", "set_autonomous_light", params);
    registerAction<ActivatePointingNode>(
      factory, "activate_pointing_action", "activate_pointing_action", params);

    factory.registerNodeType<ObstacleEndPointDetected>("obstacle_endPoint_detected", params);
    factory.registerNodeType<WaypointReached>("waypoint_reached", params);
    factory.registerNodeType<RecordCurrentStateNode>("record_current_state_action", params);
    factory.registerNodeType<RecoveryPlaceholderNode>("gpsNav_recovery_action", params);
    factory.registerNodeType<RecoveryPlaceholderNode>("imgDetection_recovery_action", params);
    factory.registerNodeType<RecoveryPlaceholderNode>("lane_follower_recovery_action", params);
    factory.registerNodeType<RecoveryPlaceholderNode>("mppi_recovery_action", params);
  }

  void setBlackboardValues(BT::Tree& tree)
  {
    auto bb = tree.rootBlackboard();
    bb->set("wp1_lat", get_parameter("wp1_lat").as_double());
    bb->set("wp1_lon", get_parameter("wp1_lon").as_double());
    bb->set("wp2_lat", get_parameter("wp2_lat").as_double());
    bb->set("wp2_lon", get_parameter("wp2_lon").as_double());
    bb->set("wp3_lat", get_parameter("wp3_lat").as_double());
    bb->set("wp3_lon", get_parameter("wp3_lon").as_double());
    bb->set("end_point_lat", get_parameter("end_point_lat").as_double());
    bb->set("end_point_lon", get_parameter("end_point_lon").as_double());
    bb->set("given_img", get_parameter("given_img").as_string());
    bb->set("mode", std::string("flashing"));
    bb->set("img_found", false);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTExecutorNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
