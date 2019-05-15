#!/usr/bin/env python

import rospy
import serial
import time

from serial import SerialException
from std_msgs.msg import Byte


def main():
    buttondown_pub = rospy.Publisher(
        '/rfreceiver/buttondown',
        Byte,
        queue_size=1
    )
    rospy.init_node('rfreceiver')

    device_path = rospy.get_param('~device_path')
    baud_rate = rospy.get_param('~baud_rate', 9600)
    retry_grace_time = rospy.get_param('~retry_grace_time', 5)
    device_timeout = rospy.get_param('~serial_timeout', 10)
    retry_attempts = rospy.get_param('~retry_attempts', 20)

    ready = False

    # Try `retry_attempts` times with `retry_grace_time` grace period between attempts to open the device

    while not ready and retry_attempts >= 0:
        try:
            receiver = serial.Serial(port=device_path, baudrate=baud_rate, timeout=device_timeout)
            ready = True
        except SerialException:
            rospy.logerr("Could not open device_path: %s in %s seconds" % (device_path, device_timeout))
            rospy.logerr("I'm going to try %s times more and will sleep %s seconds between attempts" % (retry_attempts, retry_grace_time))
            time.sleep(retry_grace_time)
            retry_attempts -= 1

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
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
