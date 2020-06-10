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
  Subs["config/decision_maker"] =
      nh_.subscribe("config/decision_maker", 3, &DecisionMakerNode::callbackFromConfig, this);

  Subs["state_cmd"] = nh_.subscribe("state_cmd", 1, &DecisionMakerNode::callbackFromStateCmd, this);
  Subs["current_velocity"] =
      nh_.subscribe("current_velocity", 1, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["obstacle_waypoint"] =
      nh_.subscribe("obstacle_waypoint", 1, &DecisionMakerNode::callbackFromObstacleWaypoint, this);
  Subs["stopline_waypoint"] =
      nh_.subscribe("stopline_waypoint", 1, &DecisionMakerNode::callbackFromStoplineWaypoint, this);
  Subs["change_flag"] = nh_.subscribe("change_flag", 1, &DecisionMakerNode::callbackFromLaneChangeFlag, this);
  Subs["lanelet_map"] = nh_.subscribe("lanelet_map_bin", 1, &DecisionMakerNode::callbackFromLanelet2Map, this);
}

void DecisionMakerNode::createPublisher(void)
{
  // pub
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("state/stopline_wpidx", 1, false);

  // for controlling other planner
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::TrafficLight>("light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::LampCmd>("lamp_cmd", 1);

  // for visualize status
  Pubs["state"] = private_nh_.advertise<std_msgs::String>("state", 1, true);
  Pubs["state_msg"] = private_nh_.advertise<autoware_msgs::State>("state_msg", 1, true);
  Pubs["state_overlay"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("state_overlay", 1);
  Pubs["available_transition"] = private_nh_.advertise<std_msgs::String>("available_transition", 1, true);
  Pubs["stop_cmd_location"] = private_nh_.advertise<autoware_msgs::VehicleLocation>("stop_location", 1, true);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("target_velocity_array", 1);
  Pubs["operator_help_text"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("operator_help_text", 1, true);
}

void DecisionMakerNode::initROS()
{

  // for subscribe callback function
  createSubscriber();
  createPublisher();

  if (disuse_vector_map_)
  {
    ROS_WARN("Disuse vectormap mode.");
  }
  else
  {
    if(use_lanelet_map_)
    {
      initLaneletMap();
    }
    else
    {
      initVectorMap();
    }
  }

  spinners = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(3));
  spinners->start();

  update_msgs();
}

void DecisionMakerNode::initLaneletMap(void)
{
  bool ll2_map_loaded = false;
  while (!ll2_map_loaded && ros::ok())
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(2, "Subscribing to lanelet map topic");
    ll2_map_loaded = isEventFlagTrue("lanelet2_map_loaded");
    ros::Duration(0.1).sleep();
  }
}

void DecisionMakerNode::initVectorMap(void)
{
  int _index = 0;
  bool vmap_loaded = true; // false;

  while (!vmap_loaded)
  {
    // Map must be populated before setupStateCallback() is called
    // in DecisionMakerNode constructor
    g_vmap.subscribe( nh_, Category::POINT | Category::LINE | Category::VECTOR |
      Category::AREA | Category::STOP_LINE | Category::ROAD_SIGN | Category::CROSS_ROAD,
      ros::Duration(1.0));

    vmap_loaded =
        g_vmap.hasSubscribed(Category::POINT | Category::LINE | Category::AREA |
                              Category::STOP_LINE | Category::ROAD_SIGN);

    if (!vmap_loaded)
    {
      ROS_WARN_THROTTLE(5, "Necessary vectormap topics have not been published.");
      ROS_WARN_THROTTLE(5, "DecisionMaker will wait until the vectormap has been loaded.");
    }
    else
    {
      ROS_INFO("Vectormap loaded.");
    }
  }

  const std::vector<CrossRoad> crossroads = g_vmap.findByFilter([](const CrossRoad& crossroad) { return true; });
  if (crossroads.empty())
  {
    ROS_INFO("crossroads have not found\n");
    return;
  }

  for (const auto& cross_road : crossroads)
  {
    geometry_msgs::Point _prev_point;
    Area area = g_vmap.findByKey(Key<Area>(cross_road.aid));
    CrossRoadArea carea;
    carea.id = _index++;
    carea.area_id = area.aid;

    double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
    double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
    double z = 0.0;
    int points_count = 0;

    const std::vector<Line> lines =
        g_vmap.findByFilter([&area](const Line& line) { return area.slid <= line.lid && line.lid <= area.elid; });
    for (const auto& line : lines)
    {
      const std::vector<Point> points =
          g_vmap.findByFilter([&line](const Point& point) { return line.bpid == point.pid; });
      for (const auto& point : points)
      {
        geometry_msgs::Point _point;
        _point.x = point.ly;
        _point.y = point.bx;
        _point.z = point.h;

        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
          continue;

        _prev_point = _point;
        points_count++;
        carea.points.push_back(_point);

        // calc a centroid point and about intersects size
        x_avg += _point.x;
        y_avg += _point.y;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
        z = _point.z;
      }  // points iter
    }    // line iter
    carea.bbox.pose.position.x = x_avg / (double)points_count * 1.5 /* expanding rate */;
    carea.bbox.pose.position.y = y_avg / (double)points_count * 1.5;
    carea.bbox.pose.position.z = z;
    carea.bbox.dimensions.x = x_max - x_min;
    carea.bbox.dimensions.y = y_max - y_min;
    carea.bbox.dimensions.z = 2;
    carea.bbox.label = 1;
    intersects.push_back(carea);
  }
}
}
