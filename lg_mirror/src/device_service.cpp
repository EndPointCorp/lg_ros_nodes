// ROS includes
#include "ros/ros.h"
#include "lg_msg_defs/EvdevDeviceInfo.h"

// Linux includes
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/types.h>
#include <sys/stat.h>

// STL includes
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <unistd.h>

#include "device_service.h"

using namespace std;

int test_bit(const char *bitmask, int bit)
{
	return bitmask[bit / 8] & (1 << (bit % 8));
}

DeviceServicer::DeviceServicer(int f)
{
	fd = f;
}

bool DeviceServicer::get_device_info(lg_msg_defs::EvdevDeviceInfo::Request &req,
                     lg_msg_defs::EvdevDeviceInfo::Response &res)
{
	struct input_id iid;
	struct input_absinfo abs;
	char name[UINPUT_MAX_NAME_SIZE] = "Unknown";
	char ev_bits[EV_MAX / 8 + 1], code_bits[KEY_MAX / 8 + 1];

	bzero(&iid, sizeof(iid));
	bzero(&abs, sizeof(abs));

	// check file descriptor...
	// maybe some kind of error should be logged
	if (fd <= 0)
		return false;

	if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 1)
		perror("name error");
	res.name = name;

	if (ioctl(fd, EVIOCGID, &iid) < 0)
		perror("id error");
	res.version = iid.version;
	res.product = iid.product;
	res.vendor = iid.vendor;
	res.bustype = iid.bustype;

	// I think this 0th bit events marks the other events that exist
	if (ioctl(fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) < 0)
		return false;

	for (int ev_type = 0; ev_type < EV_MAX; ev_type++) { 
		// just continue if this bit isn't set
		if (!test_bit(ev_bits, ev_type))
			continue;

		bzero(code_bits, sizeof(code_bits));
		// grab event bits for the current ev_type
		if (ioctl(fd, EVIOCGBIT(ev_type, sizeof(code_bits)), code_bits) < 0)
			continue;
		res.types.push_back(ev_type);
		// loop over each bit of the code_bits
		for (int ev_code = 0; ev_code < sizeof(code_bits) * 8; ev_code++) {
			// ignore if current bit isn't set
			if (!test_bit(code_bits, ev_code))
				continue;
			switch (ev_type) {
			case EV_ABS:
				res.abs_codes.push_back(ev_code);
				bzero(&abs, sizeof(abs));
				// check return here
				ioctl(fd, EVIOCGABS(ev_code), &abs);
				res.abs_min.push_back(abs.minimum);
				res.abs_max.push_back(abs.maximum);
				break;
			case EV_KEY:
				res.key_codes.push_back(ev_code);
				break;
			case EV_REL:
				res.rel_codes.push_back(ev_code);
				break;
			}
		}
	}

	return true;
}
