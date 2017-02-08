#include "ros/ros.h"
#include <linux/input.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "lg_mirror/EvdevEvent.h"
#include "lg_mirror/EvdevEvents.h"
#include "constants.h"
#include "device_service.h"

using lg_mirror::TOUCH_EVENTS_TOPIC;
using lg_mirror::DEVICE_INFO_SERVICE;

const char* DEVICE_PATH_PARAM = "~device_path";
const std::size_t EVENTS_QUEUE_LENGTH = 10;

int main(int argc, char** argv) {

  /* initialize variables */
  int i = 0;
  /* initialize ros */

  ros::init(argc, argv, "lg_mirror_sender");

  ros::NodeHandle n;

  /* open the device */

  std::string device_path;
  int device_fd;

  if (ros::param::get(DEVICE_PATH_PARAM, device_path)) {
    ROS_INFO("Using device: %s", device_path.c_str());
  } else {
    ROS_ERROR("Private parameter 'device_path' must be set");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  while (true) {
    if ((device_fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK)) < 0) {
      ROS_ERROR("[%s]: Error while opening the file you specified (%s): %s",
		ros::this_node::getName().c_str(), device_path.c_str(), strerror(errno));
      ROS_ERROR("sleeping for %d seconds", i);
      ros::Duration(i).sleep();
      if (i < 30)
        i += 3;
    } else {
      break;
    }
  }

  /* set up fd polling */

  struct pollfd poll_fd;
  memset(&poll_fd, 0, sizeof(poll_fd));
  poll_fd.fd = device_fd;
  poll_fd.events = POLLIN;

  /* advertise the topic */

  ros::Publisher evdev_pub =
    n.advertise<lg_mirror::EvdevEvents>(TOUCH_EVENTS_TOPIC,
                                        EVENTS_QUEUE_LENGTH);

  /* begin relaying from the device to the topic */

  lg_mirror::EvdevEvents events_msg;

  DeviceServicer ds(device_fd);
  ros::ServiceServer service = n.advertiseService(DEVICE_INFO_SERVICE,
		  &DeviceServicer::get_device_info, &ds);
  ros::AsyncSpinner as(1);
  as.start();

  while(ros::ok()) {
    struct input_event ev;
    struct input_event *event_data = &ev;
    lg_mirror::EvdevEvent event_msg;

    /* read an event */

    int status = poll(&poll_fd, 1, -1);
    if (status == 1) {
      // device is ready to read
      int num_read = read(device_fd, event_data, sizeof(ev));

      if (num_read != sizeof(ev)) {
        ROS_ERROR("Error getting next event");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }

    } else {
      // an error has occurred
      perror("polling device");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    /* handle the event */

    if (event_data->type == EV_SYN) {
      // only publish when a syn event is read
      if (!events_msg.events.empty()) {
        evdev_pub.publish(events_msg);
        events_msg.events.clear();
      }
      ros::spinOnce();
      continue;
    }

    // non-syn events are aggregated
    event_msg.type = event_data->type;
    event_msg.code = event_data->code;
    event_msg.value = event_data->value;
    events_msg.events.push_back(event_msg);

    ROS_DEBUG(
      "got type: %d code: %d value: %d\n",
      event_msg.type, event_msg.code, event_msg.value
    );
  }
}
