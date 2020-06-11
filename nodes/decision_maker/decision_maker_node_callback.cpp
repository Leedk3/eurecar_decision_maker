#include <cmath>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TrafficLight.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace
{
// MISSION COMPLETE FLAG
static constexpr int num_of_set_mission_complete_flag = 3;
}  // namespace

namespace decision_maker
{
void DecisionMakerNode::callbackFromVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  setEventFlag("Velodyne", true);
}

// for based waypoint
void DecisionMakerNode::callbackFromLaneWaypoint(const autoware_msgs::LaneArray& msg)
{
  current_status_.based_lane_array = msg;
  setEventFlag("received_lane_waypoint", true);
}

void DecisionMakerNode::callbackFromAllPathBlocked(const std_msgs::Bool& msg)
{
  bAllPathBlocked = msg.data;
  std::cout << "bAllPathBlocked" <<bAllPathBlocked << std::endl;
}

void DecisionMakerNode::callbackFromClosestWaypoint(const std_msgs::Int32& msg)
{
  current_status_.closest_waypoint = msg.data;
  setEventFlag("received_closest_waypoint", true);
}

void DecisionMakerNode::callbackFromCurrentPose(const nav_msgs::Odometry& msg)
{
  current_status_.pose = msg.pose.pose;
}

void DecisionMakerNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg)
{
  current_status_.velocity = amathutils::mps2kmph(msg.twist.linear.x);
}



}  // namespace decision_maker
