#!/usr/bin/env python

import rospy
import serial
import time

from serial import SerialException
from std_msgs.msg import Byte
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'rfreceiver'


def main():
    buttondown_pub = rospy.Publisher(
        '/rfreceiver/buttondown',
        Byte,
        queue_size=1
    )
    rospy.init_node(NODE_NAME)

    device_path = rospy.get_param('~device_path')
    baud_rate = rospy.get_param('~baud_rate', 9600)
    retry_grace_time = rospy.get_param('~retry_grace_time', 30)

    ready = False
    i = 0
    while not ready:
        try:
            receiver = serial.Serial(port=device_path, baudrate=baud_rate)
            ready = True
        except SerialException:
            rospy.logerr("Could not open device_path: %s" % (device_path))
            rospy.logerr("I'm going to sleep %s seconds between attempts" % (i))
            time.sleep(i)
            if i < retry_grace_time:
                i += 3

    buf = ''

    while not rospy.is_shutdown():
        try:
            button = int(receiver.readline(10).strip())
        except serial.SerialException as e:
            print e
            break

        buttondown_pub.publish(Byte(button))


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
