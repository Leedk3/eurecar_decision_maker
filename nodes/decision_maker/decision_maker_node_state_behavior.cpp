#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryGlobalPathInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Behavior: State machine for behavior planning.");
  ROS_INFO("Behavior: Global path initialize.");
  Subs["base_waypoints"] = nh_.subscribe("/based/lane_waypoints_raw", 1, &DecisionMakerNode::callbackFromLaneWaypoint, this);

}

void DecisionMakerNode::updateGlobalPathInitState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_lane_waypoint"))
  {
    tryNextState("init_path");
  }
}

void DecisionMakerNode::entryCostMapBasedPlanningState(cstring_t& state_name, int status)
{ 
  ROS_INFO("Behavior: Costmap based planning state");
}

void DecisionMakerNode::updateCostMapBasedPlanningState(cstring_t& state_name, int status)
{
  if (bAllPathBlocked)
  {
    tryNextState("all_path_blocked");
  }
}

void DecisionMakerNode::entryDecelerationState(cstring_t& state_name, int status)
{
  ROS_INFO("Behavior: All path is blocked. Vehicle should find the free space.");
}

void DecisionMakerNode::updateDecelerationState(cstring_t& state_name, int status)
{
  if (!bAllPathBlocked)
  {
    tryNextState("keep_going_mode");
  }
  else
  {
    tryNextState("hybrid_planning_mode");
  }

}

void DecisionMakerNode::entryHybridAstarState(cstring_t& state_name, int status)
{
  ROS_INFO("Behavior: Vehicle is finding the free space using Hybrid A star algorithm.");
}

void DecisionMakerNode::updateHybridAstarState(cstring_t& state_name, int status)
{
  if (!bAllPathBlocked)
  {
    tryNextState("costmap_relaying_mode");
  }
  ROS_INFO("Path finding using Hybrid A star algorithm.");
  

}

}
