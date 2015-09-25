#!/usr/bin/env python

import rospy
from interactivespaces_msgs.msg import GenericMessage
from lg_replay import DevicePublisher, DeviceReplay, LgActivityException
from evdev import InputDevice


def main():
    rospy.init_node('lg_replay', anonymous=True)

    topic_name = rospy.get_param('~topic_name', None)
    device_name = rospy.get_param('~device_name', None)
    event_ecode = rospy.get_param('~event_ecode', 'EV_KEY')
    device_path = rospy.get_param('~device_path', None)

    if not topic_name or not (device_name or device_path):
        msg = "You must provide lg_activity topic name and (device name or device path)"
        rospy.logerr(msg)
        raise ROSNodeIOException(msg)

    publisher = rospy.Publisher(topic_name, GenericMessage, queue_size=10)

    device_publisher = DevicePublisher(publisher)

    if device_path:
        device = InputDevice(device_path)
        if not device_name:
            device_name = device.name
        device_replay = DeviceReplay(device_publisher, device_name, event_ecode, device=device)
    else:
        device_replay = DeviceReplay(device_publisher, device_name, event_ecode)

    device_replay.run()
    rospy.spin()

if __name__ == '__main__':
    main()
