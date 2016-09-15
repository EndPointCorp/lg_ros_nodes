#include "ros/ros.h"
#include <string>
#include <boost/format.hpp>

#include "constants.h"
#include "uinput_device.h"
#include "viewport_mapper.h"
#include "ros_event_relay.h"
#include "lg_mirror/EvdevDeviceInfo.h"

const char* DEVICE_NAME_BASE = "Virtual Touchscreen (%s)";
const char* VIEWPORT_PARAM = "viewport";

using lg_mirror::DEVICE_INFO_SERVICE;
using lg_mirror::TOUCH_ROUTE_TOPIC;

void fail() {
  ros::shutdown();
  exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {

  /* initialize ros */

  ros::init(argc, argv, "lg_mirror_receiver");

  ros::NodeHandle n("~");

  std::string viewport_name;
  std::string device_name;
  std::string viewport_geometry;
  ViewportMapper *viewport_mapper = NULL;

  /* grab parameters */

  if (!n.getParam(VIEWPORT_PARAM, viewport_name)) {
    ROS_ERROR("'viewport' parameter is required");
    fail();
  }
  device_name = str(boost::format(DEVICE_NAME_BASE) % viewport_name);

  /* discover viewport geometry */

  std::ostringstream viewport_geometry_key;
  viewport_geometry_key << "/viewport/" << viewport_name;
  if (!n.getParam(viewport_geometry_key.str(), viewport_geometry)) {
    ROS_ERROR_STREAM("Viewport " << viewport_name << " was not found");
    fail();
  }

  /* create viewport mapper */

  try {
    viewport_mapper = new ViewportMapper(device_name, viewport_geometry);
  } catch(ViewportMapperStringError e) {
    ROS_ERROR_STREAM("ViewportMapper: " << e.what());
    fail();
  }

  /* call the device_info service before creating the device */

  ros::ServiceClient client = n.serviceClient<lg_mirror::EvdevDeviceInfo>(DEVICE_INFO_SERVICE);
  lg_mirror::EvdevDeviceInfo srv;

  client.waitForExistence();
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call the device_info service");
    fail();
  }

  /* spin up the uinput device */

  UinputDevice uinput_device(device_name);
  ROS_DEBUG_STREAM("Creating device: " << device_name << " for viewport " << viewport_name);
  if (!uinput_device.Init(srv.response)) {
    ROS_ERROR("Failed to create uinput device");
    fail();
  }

  /* wait for xinput to reflect the new device */

  if (!uinput_device.WaitForXinput()) {
    ROS_ERROR("xinput query was cancelled");
    fail();
  }

  /* instantiate an event relay */

  RosEventRelay relay(n, viewport_name, uinput_device, *viewport_mapper);
  ros::Subscriber relay_sub = n.subscribe(TOUCH_ROUTE_TOPIC, 10, &RosEventRelay::HandleRouterMessage, &relay);

  /* react until termination */

  ros::spin();
}
