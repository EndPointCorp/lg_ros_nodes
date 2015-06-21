import json

import rospy
from interactivespaces_msgs.msg import GenericMessage


class SceneListener:
    def __init__(self, callback):
        self.sub = rospy.Subscriber('/director/scene', GenericMessage,
                                    self.handle_scene)
        self.callback = callback

    def handle_scene(self, msg):
        assert msg.type == 'json'
        scene = json.loads(msg.message)
        self.callback(scene)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
