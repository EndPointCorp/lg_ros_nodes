#include "uinput_device.h"

#include "ros/ros.h"
#include <linux/input.h>
#include <linux/uinput.h>
#include <linux/types.h>
#include <algorithm>
#include <fcntl.h>
#include <stdio.h>
#include <sstream>
#include <string>

#include "util.h"
#include "lg_mirror/EvdevEvent.h"
#include "lg_mirror/EvdevEvents.h"
#include "lg_mirror/EvdevDeviceInfo.h"

// emulate a typical ELO touchscreen
const __s32 MIN_TRACKING_ID = 0;
const __s32 MAX_TRACKING_ID = 65535;
const __s32 MIN_SLOT = 0;
const __s32 MAX_SLOT = 7;

UinputDeviceInitError::UinputDeviceInitError(const char* msg): msg_(msg) {}
UinputDeviceInitError::UinputDeviceInitError(const std::string& msg): msg_(msg) {}
UinputDeviceInitError::~UinputDeviceInitError() throw() {}
const char* UinputDeviceInitError::what() const throw() {
  return msg_.c_str();
}

/**
 * \brief Constructor
 * \param device_name Human-readable name for the virtual device.
 */
UinputDevice::UinputDevice(const std::string& device_name, bool translate_to_multitouch):
  fd_(-1),
  device_name_(device_name),
  translate_to_multitouch_(translate_to_multitouch)
{}

/**
 * \brief Creates the uinput device.
 * \param info Information about the source device.
 * \return true if successful.
 */
bool UinputDevice::Init(const lg_mirror::EvdevDeviceInfoResponse& info) {
  int fd;

  // open the special uinput device
  fd = open(UinputDeviceConstants::UINPUT_PATH, O_WRONLY | O_NONBLOCK);
  if (fd < 0) {
    perror("opening the uinput node");
    return false;
  }

  try {
    InitDevice_(fd, info);
  } catch (UinputDeviceInitError& e) {
    ROS_ERROR_STREAM(e.what());
    close(fd);
    return false;
  }

  // fd is now a handle for the user device
  fd_ = fd;
  return true;
}

/**
 * \brief Internal helper for creating the uinput device.
 *
 * Throws UinputDeviceInitError on failure.
 *
 * \param fd File descriptor to operate on.
 * \param info Information about the source device.
 */
void UinputDevice::InitDevice_(int fd, const lg_mirror::EvdevDeviceInfoResponse& info) {
  struct uinput_user_dev uidev;
  int status;

  // initialize the user device struct
  memset(&uidev, 0, sizeof(uidev));

  // set allowed device types and codes
  std::size_t i;
  std::size_t sz;
  sz = info.types.size();
  for (i = 0; i < sz; i++) {
    EnableCode_(fd, UI_SET_EVBIT, info.types[i]);
  }

  sz = info.key_codes.size();
  for (i = 0; i < sz; i++) {
    __u16 code = info.key_codes[i];

    if (translate_to_multitouch_ && code == BTN_LEFT) {
      code = BTN_TOUCH;
    }

    EnableCode_(fd, UI_SET_KEYBIT, code);
  }

  sz = info.rel_codes.size();
  for (i = 0; i < sz; i++) {
    EnableCode_(fd, UI_SET_RELBIT, info.rel_codes[i]);
  }

  sz = info.abs_codes.size();
  for (i = 0; i < sz; i++) {
    __u16 code = info.abs_codes[i];


    if (i > info.abs_min.size() || i > info.abs_max.size()) {
      std::ostringstream error_msg;
      error_msg << "Device info missing abs_min or abs_max for abs code " << code << " at index " << i;
      throw UinputDeviceInitError(error_msg.str());
    }
    EnableCode_(fd, UI_SET_ABSBIT, code);
    uidev.absmax[code] = info.abs_max[i];
    uidev.absmin[code] = info.abs_min[i];

    if (translate_to_multitouch_) {
      if (code == ABS_X) {
        EnableCode_(fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
        uidev.absmax[ABS_MT_POSITION_X] = info.abs_max[i];
        uidev.absmin[ABS_MT_POSITION_X] = info.abs_min[i];
      } else if (code == ABS_Y) {
        EnableCode_(fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
        uidev.absmax[ABS_MT_POSITION_Y] = info.abs_max[i];
        uidev.absmin[ABS_MT_POSITION_Y] = info.abs_min[i];
      }
    }
  }

  if (translate_to_multitouch_) {
    EnableCode_(fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
    uidev.absmax[ABS_MT_TRACKING_ID] = MAX_TRACKING_ID;
    uidev.absmin[ABS_MT_TRACKING_ID] = MIN_TRACKING_ID;
    EnableCode_(fd, UI_SET_ABSBIT, ABS_MT_SLOT);
    uidev.absmax[ABS_MT_TRACKING_ID] = MAX_SLOT;
    uidev.absmin[ABS_MT_TRACKING_ID] = MIN_SLOT;
  }

  // set the device name
  strncpy(uidev.name, device_name_.c_str(), UINPUT_MAX_NAME_SIZE);

  // set more device attributes
  uidev.id.bustype = info.bustype;
  uidev.id.vendor = info.vendor;
  uidev.id.product = info.product;
  uidev.id.version = info.version;

  // write the device information
  status = write(fd, &uidev, sizeof(uidev));
  if (status != sizeof(uidev)) {
    throw UinputDeviceInitError("Error writing to uinput device");
  }

  // create the device
  status = ioctl(fd, UI_DEV_CREATE);
  if (status != 0) {
    throw UinputDeviceInitError("Error on ioctl UI_DEV_CREATE");
  }

  ROS_DEBUG(
    "Created device: %s with vendor: %d product: %d version: %d",
    uidev.name, uidev.id.vendor, uidev.id.product, uidev.id.version
  );
}

/**
 * \brief Shortcut for enabling an event type or code during virtual device setup.
 *
 * Throws UinputDeviceInitError on failure.
 *
 * \param fd File descriptor to write to.
 * \param codeBits Bitmask describing which type of event code to enable.
 * \param code Event code to be enabled.
 */
void UinputDevice::EnableCode_(int fd, int codeBits, int code) {
  int status = ioctl(fd, codeBits, code);
  if (status != 0) {
    std::ostringstream error_msg;
    error_msg << "Failed to enable code " << codeBits << ":" << code;
    throw UinputDeviceInitError(error_msg.str());
  }
}

/**
 * \brief Waits for the virtual device to appear in the xinput list.
 *
 * Breaks if ROS is shutdown.
 *
 * \return true if the device is available.
 */
bool UinputDevice::WaitForXinput() {
  using util::exec;

  const unsigned int MAX_INTERVAL = 1000000;// usec
  unsigned int interval = 10000; // usec

  std::ostringstream cmd;
  cmd << "xinput query-state '" << device_name_ << "'";

  while (true) {
    usleep(interval);

    if (!ros::ok()) {
      return false;
    }

    int status = system(cmd.str().c_str());
    if (status == 0) {
      return true;
    }

    interval = std::max(interval * 2, MAX_INTERVAL);
  }
}

/**
 * \brief Floats the xinput device pointer.
 * \return true if successful.
 */
bool UinputDevice::FloatPointer() const {
  std::ostringstream cmd;
  cmd << "/usr/bin/xinput float '" << device_name_ << "'";

  int status = system(cmd.str().c_str());
  return status == 0;
}

/**
 * \brief Handles a ROS message containing a vector of events.
 *
 * Implicitly writes a SYN event after the incoming events.
 *
 * Shuts down ROS upon failure.
 *
 * \param msg A message describing one or more evdev events.
 */
void UinputDevice::HandleEventMessage(const lg_mirror::EvdevEvents::Ptr& msg) {
  if (fd_ < 0) {
    ROS_ERROR("Tried to handle an event message, but UinputDevice was not initialized");
    ros::shutdown();
    return;
  }

  std::size_t num_events = msg->events.size();

  if (translate_to_multitouch_) {
    for (int i = 0; i < num_events; i++) {
      lg_mirror::EvdevEvent ev = msg->events[i];

      __u16 type = ev.type;
      __u16 code = ev.code;
      __s32 value = ev.value;

      if (type == EV_KEY) {
        if (code == BTN_LEFT) {
          if (value == 1) {
            if (!WriteEvent_(EV_ABS, ABS_MT_TRACKING_ID, 1)) {
              ROS_ERROR("Error while writing an event to the device");
              ros::shutdown();
              return;
            }
          } else if (value == 0) {
            if (!WriteEvent_(EV_ABS, ABS_MT_TRACKING_ID, -1)) {
              ROS_ERROR("Error while writing an event to the device");
              ros::shutdown();
              return;
            }
          }
        }
      }
      else if (type == EV_ABS) {
        if (code == ABS_X) {
          if (!WriteEvent_(EV_ABS, ABS_MT_POSITION_X, value)) {
            ROS_ERROR("Error while writing an event to the device");
            ros::shutdown();
            return;
          }
        }
        else if (code == ABS_Y) {
          if (!WriteEvent_(EV_ABS, ABS_MT_POSITION_Y, value)) {
            ROS_ERROR("Error while writing an event to the device");
            ros::shutdown();
            return;
          }
        }
      }
    }
  }

  for (int i = 0; i < num_events; i++) {
    lg_mirror::EvdevEvent ev = msg->events[i];

    __u16 type = ev.type;
    __u16 code = ev.code;
    __s32 value = ev.value;


    if (translate_to_multitouch_) {
      if (type == EV_KEY && code == BTN_LEFT) {
        code = BTN_TOUCH;
      }
    }

    if (!WriteEvent_(type, code, value)) {
      ROS_ERROR("Error while writing an event to the device");
      ros::shutdown();
      return;
    }

  }

  // force an EV_SYN after each message
  if (!WriteEvent_(EV_SYN, SYN_REPORT, 0)) {
    ROS_ERROR("Error while writing a SYN event");
    ros::shutdown();
    return;
  }
}

/**
 * \brief Zeroes the ABS position to clear the cursor.
 */
void UinputDevice::Zero() {
  WriteEvent_(EV_ABS, ABS_X, 0);
  WriteEvent_(EV_ABS, ABS_Y, 0);
  WriteEvent_(EV_SYN, SYN_REPORT, 0);
}

/*
 * \brief Writes an event to the virtual device.
 * \param type Event type.
 * \param code Event code.
 * \param value Event value.
 * \return true if successful.
 */
bool UinputDevice::WriteEvent_(__u16 type, __u16 code, __s32 value) {
  struct input_event ev;

  memset(&ev, 0, sizeof(ev));

  ev.type = type;
  ev.code = code;
  ev.value = value;

  int num_wrote = write(fd_, &ev, sizeof(ev));

  if (num_wrote != sizeof(ev)) {
    return false;
  }

  ROS_DEBUG(
    "Wrote type: %d code: %d value: %d",
    ev.type, ev.code, ev.value
  );

  return true;
}
