#include "ros/ros.h"
#include <string>
#include <vector>

#include "lg_common/StringArray.h"
#include "ros_event_relay.h"
#include "uinput_device.h"
#include "viewport_mapper.h"

using lg_common::StringArray;
using lg_common::StringArrayPtr;

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
    ViewportMapper mapper
):
  n_(node_handle),
  viewport_name_(viewport_name),
  device_(device),
  mapper_(mapper),
  routing_(false)
{}

/**
 * \brief Handle a change in event routing.
 * \param msg List of viewports that should receive touches.
 */
void RosEventRelay::HandleRouterMessage(const StringArrayPtr& msg) {
  bool should_route = false;
  std::size_t num_viewports = msg->strings.size();
  for (std::size_t i; i < num_viewports; i++) {
    if (msg->strings[i] == viewport_name_) {
      should_route = true;
      break;
    }
  }

  if (should_route) {
    OpenRoute_();
  } else {
    CloseRoute_();
  }
}

/**
 * \brief Begin routing events to device and map to viewport.
 *
 * If viewport mapping fails, events will not be routed.
 *
 * Otherwise, this method idempotently sets the state of this relay.
 */
void RosEventRelay::OpenRoute_() {
  if (routing_) {
    return;
  }
  try {
    mapper_.Map();
  } catch(ViewportMapperExecError e) {
    ROS_ERROR_STREAM("Mapping viewport: " << e.what());
    return;
  }
  routing_ = true;
  s_ = n_.subscribe("/lg_mirror/routing", 10, &UinputDevice::HandleEventMessage, &device_);
}

/**
 * Stop routing events to device.
 *
 * Idempotently clears the state of this relay.
 */
void RosEventRelay::CloseRoute_() {
  if (!routing_) {
    return;
  }
  routing_ = false;
  s_.shutdown();
}
