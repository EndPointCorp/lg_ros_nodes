import rospy

from evdev import InputDevice, ecodes
from lg_common.helpers import rewrite_message_to_dict, find_device
from interactivespaces_msgs.msg import GenericMessage
from lg_common.logger import get_logger
logger = get_logger('replay')


class LgActivityException(Exception):
    pass


class DeviceReplay:
    """
    Initialized with device name
    Needs a publisher to publish messages
    """
    def __init__(self, publisher, device_name, event_ecode=None, device=None):
        """
        Needs to be initialized with publisher and device name which is an identifier that DeviceReplay
        will attach to. Optional parameter is a device_path mainly for testing purposes.
        """
        self.publisher = publisher
        self.device_name = device_name
        self.device = device
        if event_ecode:
            self.event_code_num = getattr(ecodes, event_ecode)
        else:
            self.event_code_num = None
        # TODO (wz): set device permissions using udev rules because otherwise this node needs sudo
        if self.device:
            self.device = device
            logger.info("Initializing device replay with device: %s" % self.device)
        else:
            try:
                device_path = find_device(self.device_name)
                self.device = InputDevice(device_path)
                logger.debug("Initialize device reader for %s" % self.device)
            except IndexError:
                logger.error("No device with name: '%s'" % self.device_name)

    def run(self):
        for event in self.device.read_loop():
            if self.event_code_num:
                if event.type == self.event_code_num:
                    publishable_event = rewrite_message_to_dict(event)
                    self.publisher.publish_event(publishable_event)
            else:
                publishable_event = rewrite_message_to_dict(event)
                self.publisher.publish_event(publishable_event)


class DevicePublisher:
    """
    Initialized with topic name and message type
    """
    def __init__(self, publisher):
        self.publisher = publisher
        logger.info("Initialized device publisher for %s" % self.publisher)

    def publish_event(self, event):
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = str(event)
        self.publisher.publish(msg)
