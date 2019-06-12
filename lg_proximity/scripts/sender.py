#!/usr/bin/env python

import os
import pty
import rospy
import serial
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'maxbotix_proximity_sensor'


def main():
    distance_pub = rospy.Publisher(
        '/proximity/distance',
        Range,
        queue_size=5
    )
    presence_pub = rospy.Publisher(
        '/proximity/presence',
        Bool,
        queue_size=5
    )
    rospy.init_node(NODE_NAME)

    device_path = rospy.get_param('~device_path', '/dev/maxbotix.0')
    baud_rate = int(rospy.get_param('~baud_rate', 57600))
    test_mode = rospy.get_param('~test_mode', False)

    i = 0
    while True:
        try:
            if test_mode:
                master, slave = pty.openpty()
                serial_name = os.ttyname(slave)
                sensor = serial.Serial(serial_name)
            else:
                sensor = serial.Serial(device_path, baud_rate)
            # if we are here, we have successfully set sensor so we break
            rospy.loginfo("%s: We have successfully connected to the serial device" % rospy.get_name())
            break
        except serial.SerialException as e:
            rospy.logwarn("%s: Error reading from serial device: %s" % (rospy.get_name(), e.message))
            rospy.logwarn("%s: Sleeping for %s seconds and then retrying connection with serial device" % (rospy.get_name(), i))
            rospy.sleep(i)
            if i < 30:
                i += 3

    buf = ''

    while not rospy.is_shutdown():
        distance = 0
        presence = 0

        try:
            char = sensor.read(1)
        except serial.SerialException as e:
            print e
            break

        buf += char
        if char == '\x0d':
            try:
                distance = int(buf[1:4])
                presence = int(buf[6])
            except ValueError as e:
                pass
            else:
                distance_msg = Range()
                distance_msg.range = distance * 0.0254
                distance_msg.header.stamp = rospy.Time.now()
                distance_pub.publish(distance_msg)

                presence_msg = Bool()
                presence_msg.data = True if presence == 1 else False
                presence_pub.publish(presence_msg)
            finally:
                buf = ''


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
