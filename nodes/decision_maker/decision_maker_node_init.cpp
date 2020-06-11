#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::init(void)
{
  initROS();
}

void DecisionMakerNode::setupStateCallback(void)
{
  /*INIT*/
  /*** state vehicle ***/
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "Init",
                           std::bind(&DecisionMakerNode::entryInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "Init",
                           std::bind(&DecisionMakerNode::updateInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "SensorInit",
                           std::bind(&DecisionMakerNode::entrySensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "SensorInit",
                           std::bind(&DecisionMakerNode::updateSensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "LocalizationInit",
                           std::bind(&DecisionMakerNode::entryLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "LocalizationInit",
                           std::bind(&DecisionMakerNode::updateLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "PlanningInit",
                           std::bind(&DecisionMakerNode::entryPlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "PlanningInit",
                           std::bind(&DecisionMakerNode::updatePlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleInit",
                           std::bind(&DecisionMakerNode::entryVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleInit",
                           std::bind(&DecisionMakerNode::updateVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleReady",
                           std::bind(&DecisionMakerNode::entryVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleReady",
                           std::bind(&DecisionMakerNode::updateVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "BatteryCharging",
                           std::bind(&DecisionMakerNode::updateBatteryChargingState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleEmergency",
                           std::bind(&DecisionMakerNode::entryVehicleEmergencyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleEmergency",
                           std::bind(&DecisionMakerNode::updateVehicleEmergencyState, this, std::placeholders::_1, 0));
  ctx_vehicle->nextState("started");


  /*** state behavior ***/
  ctx_behavior->setCallback(state_machine::CallbackType::ENTRY, "GlobalPathInit",
                           std::bind(&DecisionMakerNode::entryGlobalPathInitState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::UPDATE, "GlobalPathInit",
                           std::bind(&DecisionMakerNode::updateGlobalPathInitState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::ENTRY, "CostMapBasedPlanning",
                           std::bind(&DecisionMakerNode::entryCostMapBasedPlanningState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::UPDATE, "CostMapBasedPlanning",
                           std::bind(&DecisionMakerNode::updateCostMapBasedPlanningState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::ENTRY, "Deceleration",
                           std::bind(&DecisionMakerNode::entryDecelerationState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::UPDATE, "Deceleration",
                           std::bind(&DecisionMakerNode::updateDecelerationState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::ENTRY, "HybridAstar",
                           std::bind(&DecisionMakerNode::entryHybridAstarState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(state_machine::CallbackType::UPDATE, "HybridAstar",
                           std::bind(&DecisionMakerNode::updateHybridAstarState, this, std::placeholders::_1, 0));
  ctx_behavior->nextState("started");
  // exit callback
}

void DecisionMakerNode::createSubscriber(void)
{
  // Config subscriber
  Subs["all_path_blocked"] = nh_.subscribe("all_path_blocked", 1, &DecisionMakerNode::callbackFromAllPathBlocked, this);
  
}

void DecisionMakerNode::createPublisher(void)
{
  // pub
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("stopline_wpidx", 1, false);

  // for controlling other planner
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::TrafficLight>("light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::LampCmd>("lamp_cmd", 1);

  // for visualize status
  
  Pubs["state_msg"] = private_nh_.advertise<autoware_msgs::State>("state", 1, true);
  Pubs["state_overlay"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("state_screen", 1);
  Pubs["available_transition"] = private_nh_.advertise<std_msgs::String>("available_transition", 1, true);
  Pubs["stop_cmd_location"] = private_nh_.advertise<autoware_msgs::VehicleLocation>("stop_location", 1, true);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("target_velocity_array", 1);
  Pubs["operator_help_text"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("operator_help_text", 1, true);
}

void DecisionMakerNode::initVariables()
{
  bAllPathBlocked = false;
}

void DecisionMakerNode::initROS()
{

  // for subscribe callback function
  createSubscriber();
  createPublisher();
  initVariables();


  spinners = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(3));
  spinners->start();

  update_msgs();
}

}
