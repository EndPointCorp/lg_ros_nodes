#ifndef _ROS_EVENT_RELAY_H_
#define _ROS_EVENT_RELAY_H_

#include "ros/ros.h"
#include <string>

#include "uinput_device.h"
#include "viewport_mapper.h"
#include <lg_msg_defs/RoutedEvdevEvents.h>

class RosEventRelay {
  public:
    RosEventRelay(
      ros::NodeHandle node_handle,
      const std::string& viewport_name,
      UinputDevice device,
      ViewportMapper mapper,
      bool auto_zero
    );
    void HandleRoutedEventsMessage(const lg_msg_defs::RoutedEvdevEvents::Ptr& msg);
    void ResetIdleRemap(const ros::TimerEvent& tev);

  private:
    ros::Subscriber router_sub_;
    ros::NodeHandle n_;
    std::string viewport_name_;
    UinputDevice device_;
    ViewportMapper mapper_;
    bool auto_zero_;
    bool should_idle_remap_;
    ros::Timer idle_remap_timer_;
};

#endif // _ROS_EVENT_RELAY_H_
