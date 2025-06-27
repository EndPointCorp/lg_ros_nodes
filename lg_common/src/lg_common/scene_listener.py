import json

import rospy
from interactivespaces_msgs.msg import GenericMessage

from lg_common.logger import get_logger
logger = get_logger('scene_listener')


class SceneListener:
    def __init__(self, callback):
        self.sub = rospy.Subscriber('/director/scene', GenericMessage,
                                    self.handle_scene)
        self.callback = callback
        try:
            logger.debug("Registered scene listener with callback: %s" % self.callback.__name__)
        except AttributeError as e:
            pass

    def handle_scene(self, msg):
        assert msg.type == 'json'
        scene = None
        try:
            scene = json.loads(msg.message)
        except ValueError:
            logger.error("Invalid json in message: %s" % msg.message)
            return
        self.callback(scene)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
