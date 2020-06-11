#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::publishLampCmd(const E_Lamp& status)
{
  autoware_msgs::LampCmd lamp_msg;
  lamp_msg.header.stamp = ros::Time::now();

  switch (status)
  {
    case E_Lamp::LAMP_LEFT:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_OFF;
      break;
    case E_Lamp::LAMP_RIGHT:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_HAZARD:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_EMPTY:
    default:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_OFF;
      break;
  }
  Pubs["lamp_cmd"].publish(lamp_msg);
}

jsk_rviz_plugins::OverlayText createOverlayText(cstring_t& data, const int column)
{
  jsk_rviz_plugins::OverlayText ret;

  // message setup
  ret.width = 500;
  ret.height = 500;
  ret.top = 10 + (column * 500);
  ret.left = 10;
  ret.bg_color.r = 0;
  ret.bg_color.g = 0;
  ret.bg_color.b = 0;
  ret.bg_color.a = 0.8;

  ret.line_width = 2;
  ret.text_size = 9;
  ret.font = "DejaVu Sans Mono";
  ret.fg_color.r = 1.0;
  ret.fg_color.g = 1.0;
  ret.fg_color.b = 0.5;
  ret.fg_color.a = 0.9;

  ret.text = data;

  return ret;
}

void DecisionMakerNode::publishOperatorHelpMessage(const cstring_t& message)
{
  static std::vector<std::string> msg_log;
  static const size_t log_size = 4;

  msg_log.push_back(message);

  if (msg_log.size() >= log_size)
  {
    msg_log.erase(msg_log.begin());
  }

  std::string joined_msg;
  for (const auto& i : msg_log)
  {
    joined_msg += "> " + i + "\n";
  }
  Pubs["operator_help_text"].publish(createOverlayText(joined_msg, 0));
}

void DecisionMakerNode::update_msgs(void)
{
  if (ctx_vehicle && ctx_behavior )
  {
    // std::cout << "hello_update : " << ctx_vehicle->getStateText() << ", " << ctx_behavior->getStateText() << std::endl;
    static std::string text_vehicle_state, text_mission_state, text_behavior_state, text_motion_state;
    text_vehicle_state = ctx_vehicle->getStateText();
    text_behavior_state = ctx_behavior->getStateText();

    static std::string overlay_text;
    overlay_text = "> Vehicle:\n" + text_vehicle_state + 
                   "\n> Behavior:\n" + text_behavior_state;
    Pubs["state_overlay"].publish(createOverlayText(overlay_text, 1));

    static autoware_msgs::State state_array_msg;
    state_array_msg.header.stamp = ros::Time::now();
    state_array_msg.vehicle_state = text_vehicle_state;
    state_array_msg.behavior_state = text_behavior_state;
    Pubs["state_msg"].publish(state_array_msg);

    static std_msgs::String transition_msg;
    transition_msg.data = ctx_vehicle->getAvailableTransition(); 
    Pubs["available_transition"].publish(transition_msg);
  }
  else
  {
    std::cerr << "ctx is not found " << std::endl;
  }
}


void DecisionMakerNode::publishStoplineWaypointIdx(const int wp_idx)
{
  std_msgs::Int32 msg;
  msg.data = wp_idx;
  Pubs["state/stopline_wpidx"].publish(msg);
}

void DecisionMakerNode::publishToVelocityArray()
{
  int count = 0;
  std_msgs::Float64MultiArray msg;

  for (const auto& i : current_status_.finalwaypoints.waypoints)
  {
    msg.data.push_back(amathutils::mps2kmph(i.twist.twist.linear.x));
    if (++count >= 10)
      break;
  }
  Pubs["target_velocity_array"].publish(msg);
}
}
