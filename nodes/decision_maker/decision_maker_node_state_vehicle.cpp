#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Vehicle: State machine for vehicle system.");
}

void DecisionMakerNode::updateInitState(cstring_t& state_name, int status)
{
  static bool is_first_callback = true;

  if (!is_first_callback)
  {
    return;
  }
  ROS_INFO("Vehicle: Eurecar is initializing now");
  tryNextState("init_start");
  is_first_callback = false;
}

void DecisionMakerNode::entrySensorInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Vehicle: Sensor init start");
  Subs["lidar"] = nh_.subscribe("/merged/velodyne_points", 1, &DecisionMakerNode::callbackFromVelodyne, this);
}

void DecisionMakerNode::updateSensorInitState(cstring_t& state_name, int status)
{
  if(isEventFlagTrue("Velodyne"))
  {
    publishOperatorHelpMessage(".");
    tryNextState("sensor_is_ready");
  }
  else{
    publishOperatorHelpMessage("Please run \"velodyne package\"");
  }  

}

void DecisionMakerNode::entryLocalizationInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Vehicle: Localization init start");
  Subs["odometry"] = nh_.subscribe("/Odometry/ekf_estimated", 5, &DecisionMakerNode::callbackFromCurrentPose, this);
}

void DecisionMakerNode::updateLocalizationInitState(cstring_t& state_name, int status)
{
  if (isLocalizationConvergence(current_status_.pose.position))
  {
    publishOperatorHelpMessage(".");
    tryNextState("localization_is_ready");
  }
  else{
    publishOperatorHelpMessage("Please check \"ekf estimation package\"");
  }
}

void DecisionMakerNode::entryPlanningInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Vehicle: Current index init start");
  publishOperatorHelpMessage(".");
  Subs["closest_waypoint"] =
      nh_.subscribe("closest_waypoint", 1, &DecisionMakerNode::callbackFromClosestWaypoint, this);
}

void DecisionMakerNode::updatePlanningInitState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_closest_waypoint"))
  {
    publishOperatorHelpMessage(".");
    tryNextState("planning_is_ready");
  }
  else{
    publishOperatorHelpMessage("Cannot calculate closest waypoint");
  }
}

void DecisionMakerNode::entryVehicleInitState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("All systems is working well");
}

void DecisionMakerNode::updateVehicleInitState(cstring_t& state_name, int status)
{
  tryNextState("vehicle_is_ready");
}

void DecisionMakerNode::entryVehicleReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateVehicleReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBatteryChargingState(cstring_t& state_name, int status)
{
 }

void DecisionMakerNode::entryVehicleEmergencyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateVehicleEmergencyState(cstring_t& state_name, int status)
{
}
}
