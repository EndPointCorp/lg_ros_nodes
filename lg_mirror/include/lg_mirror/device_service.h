// ROS includes
#include "ros/ros.h"
#include "lg_mirror/EvdevDeviceInfo.h"

class DeviceServicer
{
	int fd;
	public:
	DeviceServicer(int f);
	bool get_device_info(lg_mirror::EvdevDeviceInfo::Request &req,
			lg_mirror::EvdevDeviceInfo::Response &res);
};
