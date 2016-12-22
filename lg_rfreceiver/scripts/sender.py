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
    retry_attempts = rospy.get_param('~retry_attempts', 5)

    ready = False

    # Try `retry_attempts` times with `retry_grace_time` grace period between attempts to open the device

    while not ready and retry_attempts >= 0:
        try:
            receiver = serial.Serial(port=device_path, baudrate=baud_rate)
            ready = True
        except SerialException:
            rospy.logerr("Could not open device_path: %s" % (device_path))
            rospy.logerr("I'm going to try %s times more and will sleep %s seconds between attempts" % (retry_attempts, retry_grace_time))
            time.sleep(retry_grace_time)
            retry_attempts -= 1

    if retry_attempts <= 0:
        rospy.logerr("Could not attach to serial device %s, sleeping for %s secs" % (device_path, retry_grace_time * 100))
        rospy.sleep(retry_grace_time * 100)

    buf = ''

    if not ready:
        rospy.logerr("Giving up - couldn't connect to the device %s" % device_path)
        while not rospy.is_shutdown():
            pass
    else:
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
