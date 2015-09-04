
import os
import rospy

from geometry_msgs.msg import Twist
from evdev import InputDevice, list_devices, ecodes, categorize
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import rewrite_message_to_dict


class LgActivityException(Exception):
    pass


class DeviceReplay:
    """
    Initialized with device name
    Needs a publisher to publish messages
    """
    def __init__(self, publisher, device_name, event_ecode='EV_KEY'):
        self.event_ecode = event_ecode
        self.publisher = publisher
        self.device_name = device_name
        # TODO (wz): fix permissions using udev rules
        os.system("sudo chmod 0666 /dev/input/*")
        devices = [InputDevice(fn) for fn in list_devices()]
        self.device = [device for device in devices if device.name == self.device_name ][0]
        rospy.loginfo("Initialize device reader for %s" % self.device)

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
    def __init__(self, topic_name):
        self.topic_name = topic_name
        try:
            self.publisher = rospy.Publisher(self.topic_name, GenericMessage, queue_size=10)
        except:
            msg = "Couldnt add publisher for topic_name %s" % (self.topic_name)
            rospy.logerr(msg)
            raise DevicePublisherException(msg)
        rospy.loginfo("Initialize device publisher for %s" % topic_name)

    def publish_event(self, event):
        rospy.loginfo("Publishing event %s" % event)
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = str(event)
        self.publisher.publish(msg)
