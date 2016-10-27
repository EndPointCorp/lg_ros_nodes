#!/usr/bin/env python

import rospy
import serial
import json

from std_msgs.msg import String, Bool
from interactivespaces_msgs.msg import GenericMessage


class RfidListener(object):
    def __init__(self, port, baudrate, regular_pub, debug_pub, debug_timeout=30, notify=None):
        self.port = port
        self.baudrate = baudrate
        self.regular_pub = regular_pub
        self.debug_pub = debug_pub
        self.debug_timeout = debug_timeout
        self.last_debug_message = 0
        self.last_mode = None
        self.notify = notify
        self._setup_device()

    def _setup_device(self):
        self.device = serial.Serial(port=self.port, baudrate=self.baudrate)

    def send_notification(self, msg):
        rospy.loginfo(msg)
        if not self.notify:
            return
        note = { 'title': 'rfid', 'message': msg }
        self.notify.publish(json.dumps(note))

    def handle_debug_msg(self, msg):
        """
        Debug mode toggles which topic we output the rfid on. If
        we are in debug mode then we are setting the current state
        of the system as the desired state when scanning the current
        rfid. If not, then we are changing the state of the system to
        the one corresponding to the current rfid.
        """
        if msg.data == True:
            self.last_debug_message = rospy.get_time()
        elif msg.data == False:
            self.last_debug_message = 0

    def run(self):
        while True:
            rfid = self.device.readline()
            rfid = ''.join([ch for ch in rfid if ch.isalnum()])
            # if last debug message received at least debug_timeout seconds ago...
            if self.last_debug_message + self.debug_timeout > rospy.get_time():
                self.send_notification('RFID updated, pairing with current state')
                self.debug_pub.publish(rfid)
            else:
                self.send_notification('RFID scanned, switching state...')
                self.regular_pub.publish(rfid)
            self._setup_device()


def main():
    rospy.init_node('rfid_listener', anonymous=True)

    port = rospy.get_param('~device_path', '/dev/rfid_scanner')
    baudrate = rospy.get_param('~baudrate', 9600)
    pub_topic = rospy.get_param('~pub_topic', '/rfid/uscs/scan')
    debug_pub_topic = rospy.get_param('~debug_topic', '/rfid/set')
    debug_timeout = rospy.get_param('~debug_timeout_seconds', 30)
    notification_topic = rospy.get_param('~notification_topic', '/portal_launcher/notification')

    pub = rospy.Publisher(pub_topic, String, queue_size=10)
    debug = rospy.Publisher(debug_pub_topic, String, queue_size=10)
    notify = rospy.Publisher(notification_topic, String, queue_size=10)

    # allow exception here to kill the ros node
    rfid_listener = RfidListener(port=port, baudrate=baudrate, regular_pub=pub, debug_pub=debug, notify=notify)
    rospy.Subscriber('/rfid/mode', Bool, rfid_listener.handle_debug_msg)

    rfid_listener.run()

if __name__ == '__main__':
    main()
