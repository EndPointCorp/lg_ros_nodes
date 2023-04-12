#include "ros/ros.h"
#include <string>
#include <boost/format.hpp>

#include "uinput_device.h"
#include "viewport_mapper.h"
#include "ros_event_relay.h"
#include <lg_msg_defs/EvdevDeviceInfo.h>

const char* DEVICE_NAME_BASE = "Virtual Touchscreen (%s)";
const char* VIEWPORT_PARAM = "viewport";
const char* FLOAT_POINTER_PARAM = "float_pointer";
const char* DEVICE_ID_PARAM = "device_id";
const char* TRANSLATE_TO_MULTITOUCH_PARAM = "translate_to_multitouch";
const char* AUTO_ZERO_PARAM = "auto_zero";
const char* SHOULD_FLIP_AXIS_PARAM = "should_flip_axis";
const char* X_FLIP_PARAM = "x_flip";
const char* Y_FLIP_PARAM = "y_flip";

void fail() {
  ros::shutdown();
  exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {

  /* initialize ros */

  ros::init(argc, argv, "lg_mirror_receiver");

  ros::NodeHandle n("~");

  bool float_pointer = false;
  bool translate_to_multitouch = false;
  bool auto_zero = false;
  bool should_flip_axis = false;
  int x_flip_param = 0;
  int y_flip_param = 0;
  std::string viewport_name;
  std::string device_name;
  std::string device_id;
  std::string viewport_geometry;
  ViewportMapper *viewport_mapper = NULL;

  std::stringstream device_info_service;
  std::stringstream routed_events_topic;

  /* grab parameters */

  n.param<bool>(FLOAT_POINTER_PARAM, float_pointer, false);
  n.param<bool>(AUTO_ZERO_PARAM, auto_zero, false);
  n.param<bool>(TRANSLATE_TO_MULTITOUCH_PARAM, translate_to_multitouch, false);
  n.param<bool>(SHOULD_FLIP_AXIS_PARAM, should_flip_axis, false);
  n.param<int>(X_FLIP_PARAM, x_flip_param, 0);
  n.param<int>(Y_FLIP_PARAM, y_flip_param, 0);
  if (!n.getParam(VIEWPORT_PARAM, viewport_name)) {
    ROS_ERROR("'viewport' parameter is required");
    fail();
  }
  device_name = str(boost::format(DEVICE_NAME_BASE) % viewport_name);

  n.param<std::string>(DEVICE_ID_PARAM, device_id, "default");

  device_info_service << "/lg_mirror/" << device_id << "/device_info";
  routed_events_topic << "/lg_mirror/" << device_id << "/routed_events";

  /* discover viewport geometry */

  std::ostringstream viewport_geometry_key;
  viewport_geometry_key << "/viewport/" << viewport_name;
  if (!n.getParam(viewport_geometry_key.str(), viewport_geometry)) {
    ROS_ERROR_STREAM("Viewport " << viewport_name << " was not found");
    fail();
  }

  /* create viewport mapper */

  try {
    viewport_mapper = new ViewportMapper(device_name, viewport_geometry, should_flip_axis, x_flip_param, y_flip_param);
  } catch(ViewportMapperStringError& e) {
    ROS_ERROR_STREAM("ViewportMapper: " << e.what());
    fail();
  }

  /* call the device_info service before creating the device */

  ros::ServiceClient client = n.serviceClient<lg_msg_defs::EvdevDeviceInfo>(device_info_service.str());
  lg_msg_defs::EvdevDeviceInfo srv;

  client.waitForExistence();
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call the device_info service");
    fail();
  }

  /* spin up the uinput device */

  UinputDevice uinput_device(device_name, translate_to_multitouch);
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

  /* float the pointer if appropriate */

  if (float_pointer) {
    if (!uinput_device.FloatPointer()) {
      ROS_ERROR("failed to float pointer");
      fail();
    }
  }

	/* initial map to viewport */

	viewport_mapper->Map();

  /* instantiate an event relay */

  RosEventRelay relay(n, viewport_name, uinput_device, *viewport_mapper, auto_zero);
  ros::Subscriber events_sub = n.subscribe(routed_events_topic.str(), 10, &RosEventRelay::HandleRoutedEventsMessage, &relay);

  /* react until termination */

  ros::spin();
}
