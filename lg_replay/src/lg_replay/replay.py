import os
import rospy

from geometry_msgs.msg import Twist
from evdev import InputDevice, list_devices, ecodes, categorize
from lg_common.helpers import rewrite_message_to_dict
from interactivespaces_msgs.msg import GenericMessage


class LgActivityException(Exception):
    pass


class DeviceReplay:
    """
    Initialized with device name
    Needs a publisher to publish messages
    """
    def __init__(self, publisher, device_name, event_ecode='EV_KEY', device=None):
        """
        Needs to be initialized with publisher and device name which is an identifier that DeviceReplay
        will attach to. Optional parameter is a device_path mainly for testing purposes.
        """
        self.event_ecode = event_ecode
        self.publisher = publisher
        self.device_name = device_name
        self.device = device
        # TODO (wz): set device permissions using udev rules because otherwise this node needs sudo
        if self.device:
            self.device = device
            rospy.loginfo("Initializing device replay with devic: %s" % self.device)
        else:
            try:
                os.system("sudo chmod 0666 /dev/input/*")
                devices = [InputDevice(fn) for fn in list_devices()]
                self.device = [device for device in devices if device.name == self.device_name][0]
                rospy.loginfo("Initialize device reader for %s" % self.device)
            except IndexError, e:
                rospy.logerr("No device with name: '%s'" % self.device_name)

    def run(self):
        for event in self.device.read_loop():
            rospy.loginfo("Catched event: %s" % event)
            if self.event_ecode:
                if event.type == getattr(ecodes, self.event_ecode):
                    event = categorize(event)
                    publishable_event = rewrite_message_to_dict(event)
                    self.publisher.publish_event(publishable_event)
            else:
                event = categorize(event)
                publishable_event = rewrite_message_to_dict(event)
                self.publisher.publish_event(publishable_event)


class DevicePublisher:
    """
    Initialized with topic name and message type
    """
    def __init__(self, publisher):
        self.publisher = publisher
        rospy.loginfo("Initialized device publisher for %s" % self.publisher)

    def publish_event(self, event):
        rospy.loginfo("Publishing event %s" % event)
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = str(event)
        self.publisher.publish(msg)
