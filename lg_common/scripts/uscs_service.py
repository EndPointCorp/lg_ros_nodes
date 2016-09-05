#!/usr/bin/env python

import roslib
roslib.load_manifest('lg_common')

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
ON_OFFLINE_STATE = "http://lg-head:8088/director_api/on_offline_scene"
ON_ONLINE_STATE = "http://lg-head:8088/director_api/on_online_scene"
ON_INACTIVE_STATE = "http://lg-head:8088/director_api/on_inactive_scene"
ON_ACTIVE_STATE = "http://lg-head:8088/director_api/on_active_scene"


class USCSState:

    def __init__(self,
                 initial_state_scene_url='',
                 on_online_state_scene_url='',
                 on_offline_state_scene_url='',
                 on_active_state_scene_url='',
                 on_inactive_state_scene_url='',
                 history_size=20):
        """
        Accepts different states URLs:
         - on_online_state emitted when
        """
        self.history_size = history_size
        self.history = []
        self.on_online_state = self._grab_scene(on_online_state_scene_url)
        self.on_offline_state = self._grab_scene(on_offline_state_scene_url)
        self.on_active_state = self._grab_scene(on_active_state_scene_url)
        self.on_inactive_state = self._grab_scene(on_inactive_state_scene_url)

        self.state = self._grab_state(initial_state)

        if self.state is None:
            self.state = USCSMessageResponse()

    def handle_online_state(self, message):
        """
        Accepts online/offline state message
        and emits appropriate message from ivars
        """
        pass

    def handle_activity_state(self, message):
        """
        Accepts active/inactive state message
        and emits appropriate message from ivars
        """
        pass

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
        self._store_message_in_history(message)

    def _store_message_in_history(self, message):
        """
        Appends director scene message to history
        Keeps the history at
        """
        self.history.append(message)

    def _get_json_from_url(self, url, to_json=True):
        """
        Accepts a URL and returns json parsed version of it
        """
        response = urllib.urlopen(scene_url)
        if response.code != 200:
            rospy.sleep(3)
            return None

        try:
            message = response.read()
        except Exception:
            rospy.logwarn("Could not get response for initial state service")
            rospy.sleep(3)
            return None

        try:
            if to_json:
                message = json.loads(message)
            return message
        except ValueError:
            rospy.logwarn("invalid initial_state url: (%s)" % scene_url)
            rospy.sleep(3)
            return None

    def _create_message(self, msg_string):
        message = GenericMessage()
        message.type = 'json'
        try:
            message_json = json.loads(msg_string)
            message.message = json.dumps(message_json)
            return message
        except ValueError:
            print "Could not decode json message for msg_string: %s" % msg_string
            sys.exit(1)

    def _grab_scene(self, scene_url):
        """
        Gets json from a url and turns it into GenericMessage from interactivespaces messages
        """
        if not_scene_url:
            return None

        message = self._get_json_from_url(scene_url, to_json=False)
        message = self._create_message(message)
        return message

    def _grab_state(self, scene_url):
        """
        Curls for scene and parses the json. If
        all goes well it becomes the initial state,
        otherwise the initial state will be none
        """
        if not scene_url:
            return None

        message = self._get_json_from_url(scene_url)

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
