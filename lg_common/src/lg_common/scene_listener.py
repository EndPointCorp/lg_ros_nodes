import json

import rospy
from lg_common.helpers import write_log_to_file
from interactivespaces_msgs.msg import GenericMessage


class SceneListener:
    def __init__(self, callback):
        self.sub = rospy.Subscriber('/director/scene', GenericMessage,
                                    self.handle_scene)
        write_log_to_file("Starting SceneListener from %s" % self.__class__)
        self.callback = callback
        try:
            write_log_to_file("Registered SceneListener from %s" % self.__class__)
            rospy.loginfo("Registered scene listener with callback: %s" % self.callback.__name__)
        except AttributeError, e:
            pass

    def handle_scene(self, msg):
        assert msg.type == 'json'
        scene = None
        try:
            write_log_to_file("Trying to load json message")
            scene = json.loads(msg.message)
        except ValueError:
            write_log_to_file("Failed to load json message")
            rospy.logerr("Invalid json in message: %s" % msg.message)
            return
        self.callback(scene)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
