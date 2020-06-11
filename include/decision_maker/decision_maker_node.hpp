#ifndef __DECISION_MAKER_NODE__
#define __DECISION_MAKER_NODE__

#include <unordered_map>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <random>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_config_msgs/ConfigDecisionMaker.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/State.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleLocation.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/WaypointState.h>
#include <vector_map/vector_map.h>

#include <amathutils_lib/amathutils.hpp>
#include <cross_road_area.hpp>
#include <decision_maker_param.hpp>
#include <state_machine_lib/state_context.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/Route.h>

#include <autoware_lanelet2_msgs/MapBin.h>

namespace decision_maker
{
using namespace vector_map;
using cstring_t = const std::string;

enum class E_Lamp : int32_t
{
  LAMP_EMPTY = -1,
  LAMP_CLEAR = 0,
  LAMP_RIGHT = 1,
  LAMP_LEFT = 2,
  LAMP_HAZARD = 3
};
enum class E_Control : int32_t
{
  KEEP = -1,
  STOP = 1,
  DECELERATE = 2,
  ACCELERATE = 3,
  OTHERS = 4,
};

enum class E_ChangeFlags : int32_t
{
  STRAIGHT,
  RIGHT,
  LEFT,

  UNKNOWN = -1,
};

inline bool hasvMap(void)
{
  return true;
}

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

struct AutowareStatus
{
  std::map<std::string, bool> EventFlags;

  // planning status
  autoware_msgs::LaneArray using_lane_array;  // with wpstate
  autoware_msgs::LaneArray based_lane_array;
  autoware_msgs::Lane finalwaypoints;
  int closest_waypoint;
  int obstacle_waypoint;
  int stopline_waypoint;
  int change_flag;

  // vehicle status
  geometry_msgs::Pose pose;
  double velocity;  // kmph

  int found_stopsign_idx;
  int prev_stopped_wpidx;
  int ordered_stop_idx;
  int prev_ordered_idx;

  AutowareStatus(void) : closest_waypoint(-1), obstacle_waypoint(-1), stopline_waypoint(-1), velocity(0), found_stopsign_idx(-1), prev_stopped_wpidx(-1), ordered_stop_idx(-1), prev_ordered_idx(-1)
  {
  }

  // control status
};

class DecisionMakerNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Publishers
  std::unordered_map<std::string, ros::Publisher> Pubs;
  // Subscribers
  std::unordered_map<std::string, ros::Subscriber> Subs;

  std::shared_ptr<ros::AsyncSpinner> spinners;

  AutowareStatus current_status_;

  std::vector<CrossRoadArea> intersects;

  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;

  class DetectionArea
  {
  public:
    double x1, x2;
    double y1, y2;

    DetectionArea()
    {
    }
  };
  DetectionArea detectionArea_;

  //behavior state
  bool bAllPathBlocked;




  // initialization method
  void initROS();

  void createSubscriber(void);
  void createPublisher(void);
  void initVariables();

  // looping method
  void update(void);
  void update_msgs(void);

  void publishToVelocityArray();

  void publishOperatorHelpMessage(const cstring_t& message);
  void publishLampCmd(const E_Lamp& status);
  void publishStoplineWaypointIdx(const int wp_idx);


  /* decision */
  void tryNextState(cstring_t& key);
  bool isLocalizationConvergence(const geometry_msgs::Point& _current_point) const;
  bool waitForEvent(cstring_t& key, const bool& flag);
  bool waitForEvent(cstring_t& key, const bool& flag, const double& timeout);

  double calcIntersectWayAngle(const autoware_msgs::Lane& laneinArea);
  double getDistToWaypointIdx(const int wpidx) const;
  double calcRequiredDistForStop(void) const;


  void setupStateCallback(void);

  /*
   * state callback
   **/

  /*** state vehicle ***/
  // entry callback
  void entryInitState(cstring_t& state_name, int status);
  void entrySensorInitState(cstring_t& state_name, int status);
  void entryLocalizationInitState(cstring_t& state_name, int status);
  void entryPlanningInitState(cstring_t& state_name, int status);
  void entryVehicleInitState(cstring_t& state_name, int status);
  void entryVehicleReadyState(cstring_t& state_name, int status);
  void entryVehicleEmergencyState(cstring_t& state_name, int status);
  // update callback
  void updateInitState(cstring_t& state_name, int status);
  void updateSensorInitState(cstring_t& state_name, int status);
  void updateLocalizationInitState(cstring_t& state_name, int status);
  void updatePlanningInitState(cstring_t& state_name, int status);
  void updateVehicleInitState(cstring_t& state_name, int status);
  void updateVehicleReadyState(cstring_t& state_name, int status);
  void updateBatteryChargingState(cstring_t& state_name, int status);
  void updateVehicleEmergencyState(cstring_t& state_name, int status);
  // exit callback

  /*** state behavior ***/
  // entry callback
  void entryGlobalPathInitState(cstring_t& state_name, int status);
  void entryCostMapBasedPlanningState(cstring_t& state_name, int status);
  void entryDecelerationState(cstring_t& state_name, int status);
  void entryHybridAstarState(cstring_t& state_name, int status);
  // update callback
  void updateGlobalPathInitState(cstring_t& state_name, int status);
  void updateCostMapBasedPlanningState(cstring_t& state_name, int status);
  void updateDecelerationState(cstring_t& state_name, int status);
  void updateHybridAstarState(cstring_t& state_name, int status);
  // exit callback


  // callback by topic subscribing
  void callbackFromVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg);
  void callbackFromCurrentPose(const nav_msgs::Odometry& msg);
  void callbackFromClosestWaypoint(const std_msgs::Int32& msg);
  void callbackFromLightColor(const ros::MessageEvent<autoware_msgs::TrafficLight const>& event);

  void callbackFromLaneWaypoint(const autoware_msgs::LaneArray& msg);
  void callbackFromAllPathBlocked(const std_msgs::Bool& msg);

  void setEventFlag(cstring_t& key, const bool& value)
  {
    current_status_.EventFlags[key] = value;
  }

  bool isEventFlagTrue(std::string key)
  {
    if (current_status_.EventFlags.count(key) == 0)
    {
      current_status_.EventFlags[key] = false;
    }
    return current_status_.EventFlags[key];
  }

public:
  state_machine::StateContext* ctx_vehicle;
  state_machine::StateContext* ctx_behavior;
  VectorMap g_vmap;

  DecisionMakerNode(int argc, char** argv)
    : private_nh_("~")
  {
    std::string file_name_vehicle;
    std::string file_name_behavior;
    private_nh_.getParam("state_vehicle_file_name", file_name_vehicle);
    private_nh_.getParam("state_behavior_file_name", file_name_behavior);

    current_status_.prev_stopped_wpidx = -1;

    ctx_vehicle = new state_machine::StateContext(file_name_vehicle, "eurecar_states_vehicle");
    ctx_behavior = new state_machine::StateContext(file_name_behavior, "eurecar_states_behavior");

    init();
    setupStateCallback();
  }

  void init(void);
  void run(void);

  bool isSubscriberRegistered(cstring_t& topic_name)
  {
    return Subs.count(topic_name) ? true : false;
  }

  static geometry_msgs::Point VMPoint2GeoPoint(const vector_map_msgs::Point& vp)
  {
    geometry_msgs::Point gp;
    gp.x = vp.ly;
    gp.y = vp.bx;
    gp.z = vp.h;
    return gp;
  }
};

}  // namespace decision_maker

#endif
