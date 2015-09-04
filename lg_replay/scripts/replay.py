#!/usr/bin/env python

import rospy
from lg_replay import DevicePublisher, DeviceReplay, LgActivityException


def main():
    rospy.init_node('lg_replay', anonymous=True)

    topic_name = rospy.get_param('~topic_name', None)
    device_name = rospy.get_param('~device_name', None)
    event_ecode = rospy.get_param('~event_ecode', 'EV_KEY')

    if not topic_name or not device_name:
        msg = "You must provide lg_activity topic name and device name"
        rospy.logerr(msg)
        raise ROSNodeIOException(msg)

    publisher = DevicePublisher(topic_name)
    device_replay = DeviceReplay(publisher, device_name, event_ecode)
    device_replay.run()
    rospy.spin()

if __name__ == '__main__':
    main()
