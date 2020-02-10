#ifndef _UINPUT_DEVICE_H_
#define _UINPUT_DEVICE_H_

#include <linux/types.h>
#include <string>
#include <exception>

#include <lg_msg_defs/EvdevEvents.h>
#include <lg_msg_defs/EvdevDeviceInfo.h>

namespace UinputDeviceConstants {
  const char* UINPUT_PATH = "/dev/uinput";
}

class UinputDeviceInitError : public std::exception {
  public:
    explicit UinputDeviceInitError(const char* msg);
    explicit UinputDeviceInitError(const std::string& msg);
    virtual ~UinputDeviceInitError() throw();
    virtual const char* what() const throw();
  protected:
    std::string msg_;
};

class UinputDevice {
  public:
    UinputDevice(const std::string& device_name, bool translate_to_multitouch);

    bool Init(const lg_msg_defs::EvdevDeviceInfoResponse& info);
    bool WaitForXinput();
    bool FloatPointer() const;
    void Zero();

    void HandleEventMessage(const lg_msg_defs::EvdevEvents::Ptr& msg);

  private:
    static void EnableCode_(int fd, int codeBits, int code);
    void InitDevice_(int fd, const lg_msg_defs::EvdevDeviceInfoResponse& info);
    bool WriteEvent_(__u16 type, __u16 code, __s32 value);

    int fd_;
    std::string device_name_;
    bool translate_to_multitouch_;
    __s32 offscreen_x_;
    __s32 offscreen_y_;
};

#endif // _UINPUT_DEVICE_H_
