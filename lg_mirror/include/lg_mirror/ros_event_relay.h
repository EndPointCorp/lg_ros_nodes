#ifndef _ROS_EVENT_RELAY_H_
#define _ROS_EVENT_RELAY_H_

#include "ros/ros.h"
#include <string>

#include "uinput_device.h"
#include "viewport_mapper.h"
#include "lg_common/StringArray.h"

class RosEventRelay {
  public:
    RosEventRelay(
      ros::NodeHandle node_handle,
      const std::string& viewport_name,
      UinputDevice device,
      const std::string& events_topic,
      ViewportMapper mapper,
      bool auto_zero
    );
    void HandleRouterMessage(const lg_common::StringArrayPtr& msg);

  private:
    void OpenRoute_();
    void CloseRoute_();
    void ZeroDevice_();

    bool routing_;
    ros::Subscriber s_;
    ros::NodeHandle n_;
    std::string viewport_name_;
    std::string events_topic_;
    UinputDevice device_;
    ViewportMapper mapper_;
    bool auto_zero_;
};

#endif // _ROS_EVENT_RELAY_H_
