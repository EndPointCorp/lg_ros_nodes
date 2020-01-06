// ROS includes
#include "ros/ros.h"
#include <lg_msg_defs/EvdevDeviceInfo.h>

class DeviceServicer
{
	int fd;
	public:
	DeviceServicer(int f);
	bool get_device_info(lg_msg_defs::EvdevDeviceInfo::Request &req,
			lg_msg_defs::EvdevDeviceInfo::Response &res);
};
