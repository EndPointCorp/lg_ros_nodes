#!/usr/bin/env python

import rospy
from rosnode import ROSNodeIOException
from lg_common.helpers import get_params
from interactivespaces_msgs.msg import GenericMessage
from lg_replay import DevicePublisher, DeviceReplay, LgActivityException
from evdev import InputDevice
import os


def main():
    rospy.init_node('lg_replay', anonymous=True)

    topic_name = get_params('~topic_name', None)
    device_name = get_params('~device_name', None)
    event_ecode = get_params('~event_ecode', 'EV_KEY')
    device_path = get_params('~device_path', None)

    if not topic_name or not (device_name or device_path):
        msg = "You must provide lg_activity topic name and (device name or device path)"
        rospy.logerr(msg)
        raise ROSNodeIOException(msg)

    publisher = rospy.Publisher(topic_name, GenericMessage, queue_size=10)

    device_publisher = DevicePublisher(publisher)

    if device_path:
        err, msg = check_device_path(device_path)
        if err:
            rospy.logerr('Invalid device path supplied: %s' % msg)
            return
        device = InputDevice(device_path)
        if not device_name:
            device_name = device.name
        device_replay = DeviceReplay(device_publisher, device_name, event_ecode, device=device)
    else:
        device_replay = DeviceReplay(device_publisher, device_name, event_ecode)

    try:
        device_replay.run()
    except IOError:
        rospy.logwarn('Device unplugged most likely')


def check_device_path(path):
    if not os.path.exists(path):
        return True, 'Device does not exist'
    err = not os.access(path, os.R_OK | os.W_OK)
    if err:
        return err, 'Insufficient permissions'
    return False, 'No problems here'

if __name__ == '__main__':
    main()
