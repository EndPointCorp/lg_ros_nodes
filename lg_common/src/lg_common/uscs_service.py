#!/usr/bin/env python

import rospy
import urllib
import json

from interactivespaces_msgs.msg import GenericMessage
from lg_common.srv import USCSMessage, USCSMessageResponse, InitialUSCS, InitialUSCSResponse


class USCSService:

    def __init__(self,
                 initial_state_scene_url='',
                 on_online_state_scene_url='',
                 on_offline_state_scene_url='',
                 on_active_state_scene_url='',
                 on_inactive_state_scene_url='',
                 director_scene_publisher=None):
        """
        Accepts different states URLs:
         - on_online_state emitted when
        """
        self.director_scene_publisher = director_scene_publisher
        self.history = []

        if on_online_state_scene_url:
            self.on_online_state = self._grab_scene(on_online_state_scene_url)
            rospy.loginfo("Enabling on_online_state handling")
        else:
            rospy.loginfo("Disabling on_online_state handling")
            self.on_online_state = None

        if on_offline_state_scene_url:
            rospy.loginfo("Enabling on_offline_state handling")
            self.on_offline_state = self._grab_scene(on_offline_state_scene_url)
        else:
            rospy.loginfo("Disabling on_offline_state handling")
            self.on_offline_state = None

        if on_active_state_scene_url:
            rospy.loginfo("Enabling on_active_state handling")
            self.on_active_state = self._grab_scene(on_active_state_scene_url)
        else:
            rospy.loginfo("Disabling on_active_state handling")
            self.on_active_state = None

        if on_inactive_state_scene_url:
            rospy.loginfo("Enabling on_inactive_state handling")
            self.on_inactive_state = self._grab_scene(on_inactive_state_scene_url)
        else:
            rospy.loginfo("Disabling on_inactive_state handling")
            self.on_inactive_state = None

        self.state = self._grab_state(initial_state_scene_url)

        if self.state is None:
            self.state = USCSMessageResponse()

    def handle_connectivity_message(self, message):
        """
        Accepts online/offline state message
        and emits appropriate message from ivars

        If data == True then emit on_online_state message
        If data == False then emit on_offline_state message
        """
        if self.director_scene_publisher:
            if message.data is True and self.on_online_state:
                """
                We became online
                """
                self.director_scene_publisher.publish(self.on_online_state)
            if message.data is False and self.on_offline_state:
                """
                We became offline
                """
                self.director_scene_publisher.publish(self.on_offline_state)

    def handle_activity_message(self, message):
        """
        Accepts active/inactive state message
        and emits appropriate message from ivars
        """
        if self.director_scene_publisher:
            if message.data is True and self.on_active_state:
                """
                We became active
                """
                self.director_scene_publisher.publish(self.on_active_state)
            if message.data is False and self.on_inactive_state:
                """
                We became inactive
                """
                self.director_scene_publisher.publish(self.on_inactive_state)

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
        return InitialUSCSResponse(state.type, json.dumps(state.message))

    def update_uscs_message(self, message):
        """
        Subscribed to track the current state
        """
        rospy.loginfo("Getting message {}".format(message))
        self.state = message

    def _get_json_from_url(self, url, to_json=True):
        """
        Accepts a URL and returns json parsed version of it
        """
        response = urllib.urlopen(url)
        if response.code != 200:
            rospy.logwarn("Got non-200 error status (%s) from url: %s" % (response.code, url))
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
        if not scene_url:
            return None

        try:
            message = self._get_json_from_url(scene_url, to_json=False)
            message = self._create_message(message)
        except Exception, e:
            message = None
            rospy.logwarn("Could not get scene for url: %s because %s" % (scene_url, e))
            rospy.logwarn("sleeping for 3 seconds")
            rospy.sleep(3)

        return message

    def _grab_state(self, scene_url):
        """
        Curls for scene and parses the json. If
        all goes well it becomes the initial state,
        otherwise the initial state will be none
        """
        if not scene_url:
            return None

        rospy.logwarn("Attempting to grab state from %s" % scene_url)
        message = self._get_json_from_url(scene_url)

        initial_state = USCSMessageResponse()
        initial_state.type = 'json'
        initial_state.message = message

        return initial_state
