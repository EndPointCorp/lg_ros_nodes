#!/usr/bin/env python

import rospy
import os
import select
import commands
from rosnode import ROSNodeIOException
from interactivespaces_msgs.msg import GenericMessage
from lg_replay import DevicePublisher, DeviceReplay
from evdev import InputDevice
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_replay'


def main():
    rospy.init_node('lg_replay', anonymous=True)

    topic_name = rospy.get_param('~topic_name', None)
    device_name = rospy.get_param('~device_name', None)
    event_ecode = rospy.get_param('~event_ecode', None)
    device_path = rospy.get_param('~device_path', None)
    user = rospy.get_param('~user', 'lg')
    group = rospy.get_param('~group', 'lg')

    if not topic_name or not (device_name or device_path):
        msg = "You must provide lg_replay output topic name and (device name or device path)"
        rospy.logerr(msg)
        raise ROSNodeIOException(msg)

    publisher = rospy.Publisher(topic_name, GenericMessage, queue_size=10)

    device_publisher = DevicePublisher(publisher)

    if device_path:
        i = 0
        while True:
            err, msg = check_device_path(device_path, user, group)
            if err:
                rospy.logerr('[%s] Invalid device path supplied, sleeping for %s seconds and trying again: %s' % (rospy.get_name(), i, msg))
            else:
                break
            rospy.sleep(i)
            if i < 30:
                i += 3
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
    except select.error, error:
        if error[0] == (4, 'Interrupted system call'):
            rospy.logwarn('Interrupted system call during waiting for event - is system shutting down?')


def check_device_path(path, user, group):
    if not os.path.exists(path):
        return True, 'Device does not exist'
    err = not os.access(path, os.R_OK | os.O_RDWR | os.O_NONBLOCK)
    if err:
        status, output = commands.getstatusoutput("sudo chown %s:%s %s" % (user, group, path))
        if status != 0:
            rospy.logerr("Could not attach to device: %s - insufficient permissions" % path)
    return False, 'No problems with lg_replay device'

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
