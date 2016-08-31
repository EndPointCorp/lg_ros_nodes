#!/usr/bin/env python

import roslib; roslib.load_manifest('lg_common');

import rospy
import urllib
import json

from std_msgs.msg import String
from lg_common.srv import USCSMessage, USCSMessageResponse, InitialUSCS, InitialUSCSResponse
from interactivespaces_msgs.msg import GenericMessage


# TODO implement this in the ros_cms side of things so
# that the initial scene can be set from the web interface
# instead of via hacky url
INITIAL_STATE = "http://lg-head:8088/director_api/initial_scene"

class USCSState:

    def __init__(self, initial_state=''):
        self.state = self._grab_scene(initial_state)
        if self.state is None:
            self.state = USCSMessageResponse()

    def current_uscs_message(self, *args, **kwargs):
        """
        Service run to tell nodes their state when they start up, this
        service should be depended on after director is up
        """
        rospy.loginfo("Current state: %s" % self.state)
        if self.state:
            #return USCSMessageResponse(self.state.type, self.state.message)
            return self.state
        return USCSMessageResponse()

    def initial_state(self, *req, **kwargs):
        """
        Just return the current state
        """
        state = self.current_uscs_message()
        return InitialUSCSResponse(state.type, state.message)

    def update_uscs_message(self, message):
        """
        Subscribed to track the current state
        """
        rospy.loginfo("Getting message {}".format(message))
        self.state = message

    def _grab_scene(self, scene_url):
        """
        Curls for scene and parses the json. If
        all goes well it becomes the initial state,
        otherwise the initial state will be none
        """
        if not scene_url:
            return None

        response = urllib.urlopen(scene_url)
        if response.code != 200:
            return None

        message = response.read()
        # if json parsing doesn't work, return none after warning
        try:
            json.loads(message)
        except ValueError:
            rospy.logwarn("invalid initial_state url: (%s)" % scene_url)
            return None

        initial_state = USCSMessageResponse()
        initial_state.type = 'json'
        initial_state.message = message

        return initial_state


def main():
    rospy.init_node('lg_uscs', anonymous=False)

    director_topic = rospy.get_param('~director_topic', '/director/scene')
    message_topic = rospy.get_param('~message_topic', '/uscs/message')
    initial_state = rospy.get_param('~initial_state', '')

    us = USCSState(initial_state=initial_state)
    rospy.Subscriber(director_topic, GenericMessage, us.update_uscs_message)
    rospy.Service(message_topic, USCSMessage, us.current_uscs_message)
    rospy.Service('/initial_state', InitialUSCS, us.initial_state)

    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
