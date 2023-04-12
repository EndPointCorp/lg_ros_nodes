#include "ros/ros.h"
#include <string>
#include <vector>
#include <linux/input.h>

#include <lg_msg_defs/StringArray.h>
#include <lg_msg_defs/EvdevEvents.h>
#include "ros_event_relay.h"
#include "uinput_device.h"
#include "viewport_mapper.h"

using lg_msg_defs::StringArray;
using lg_msg_defs::StringArrayPtr;

/**
 * \brief Constructor
 * \param node_handle ROS node handle.
 * \param viewport_name Name of the viewport we are handling.
 * \param device Uinput device awaiting event messages.
 * \param mapper Xinput mapper for viewport.
 */
RosEventRelay::RosEventRelay(
    ros::NodeHandle node_handle,
    const std::string& viewport_name,
    UinputDevice device,
    ViewportMapper mapper,
    bool auto_zero
):
  n_(node_handle),
  viewport_name_(viewport_name),
  device_(device),
  mapper_(mapper),
  auto_zero_(auto_zero),
  should_idle_remap_(true)
{}

void RosEventRelay::HandleRoutedEventsMessage(const lg_msg_defs::RoutedEvdevEvents::Ptr& msg) {
	if (std::find(msg->routes.begin(), msg->routes.end(), viewport_name_) == msg->routes.end()) {
		return;
	}
  if (should_idle_remap_) {
    ROS_DEBUG("idle remapping");
    should_idle_remap_ = false;
    try {
      mapper_.Map();
    } catch(ViewportMapperExecError& e) {
      ROS_ERROR_STREAM("Mapping viewport exec error: " << e.what());
    } catch(ViewportMapperStringError& e) {
      ROS_ERROR_STREAM("Mapping viewport string error: " << e.what());
    }
  }
  device_.HandleEventMessage(msg);
  idle_remap_timer_ = n_.createTimer(ros::Duration(1.0), &RosEventRelay::ResetIdleRemap, this, true);
}

void RosEventRelay::ResetIdleRemap(const ros::TimerEvent& tev) {
  ROS_DEBUG("reset idle remap");
  should_idle_remap_ = true;
}
