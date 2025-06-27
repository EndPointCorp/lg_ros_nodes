#!/usr/bin/env python3

import rospy
import urllib.request, urllib.parse, urllib.error
import json
import threading
import sys

from interactivespaces_msgs.msg import GenericMessage
from lg_msg_defs.srv import USCSMessage, USCSMessageResponse, InitialUSCS, InitialUSCSResponse
from std_srvs.srv import EmptyResponse

from lg_common.logger import get_logger
logger = get_logger('uscs_service')


class USCSService:

    def __init__(self,
                 initial_state_scene_url='',
                 on_online_state_scene_url='',
                 on_offline_state_scene_url='',
                 on_active_state_scene_url='',
                 on_inactive_state_scene_url='',
                 director_scene_publisher=None):
        """
        Accepts different states URLs. If URL is not set,
        message won't be emitted.

        """
        self.lock = threading.Lock()
        self.active = True
        self.offline = False

        self.director_scene_publisher = director_scene_publisher
        self.history = []

        if on_online_state_scene_url:
            self.on_online_state = self._grab_scene(on_online_state_scene_url)
            logger.debug("Enabling on_online_state handling")
        else:
            logger.debug("Disabling on_online_state handling")
            self.on_online_state = None

        if on_offline_state_scene_url:
            logger.debug("Enabling on_offline_state handling")
            self.on_offline_state = self._grab_scene(on_offline_state_scene_url)
        else:
            logger.debug("Will use initial state for on_offline_state")
            self.on_offline_state = None

        if on_active_state_scene_url:
            logger.debug("Enabling on_active_state handling")
            self.on_active_state = self._grab_scene(on_active_state_scene_url)
        else:
            logger.debug("Disabling on_active_state handling")
            self.on_active_state = None

        if on_inactive_state_scene_url:
            logger.debug("Enabling on_inactive_state handling")
            self.on_inactive_state = self._grab_scene(on_inactive_state_scene_url)
        else:
            logger.debug("Disabling on_inactive_state handling")
            self.on_inactive_state = None

        self.state = self._grab_state(initial_state_scene_url)

        if self.state is None:
            self.state = USCSMessageResponse()

        if self.on_offline_state is None:
            """
            By default, when offline, go to initial state if there is no offline state
            explicitly specifyed.
            """
            self.on_offline_state = self.state

    def handle_offline_message(self, message):
        """
        Accepts online/offline state message
        and emits appropriate message from ivars

        If data == True then emit on_offline_state message
        If data == False then emit on_online_state message
        """
        logger.debug("Incoming message: %s. Currently offline: %s" % (message, self.offline))
        if self.director_scene_publisher and not self.offline == message.data:
            self.offline = message.data
            if self.offline is False:
                """
                We became online
                """
                self.idempotently_publish_scene(self.on_online_state)

            if self.offline is True:
                """
                We became offline
                """
                self.idempotently_publish_scene(self.on_offline_state)

    def handle_activity_message(self, message):
        """
        Accepts active/inactive state message
        and emits appropriate message from ivars
        """
        logger.debug("Incoming message: %s. Current state: %s" % (message, self.active))
        if self.director_scene_publisher:
            if (message.data is True) and (self.active is False):
                """
                We became active
                """
                logger.debug("Active False => True")
                self.idempotently_publish_scene(self.on_active_state)
                self.active = True
            if (message.data is False) and (self.active is True):
                """
                We became inactive
                """
                logger.debug("Active True => False")
                self.idempotently_publish_scene(self.on_inactive_state)
                self.active = False

    def idempotently_publish_scene(self, scene):
        """
        Attempt to publish a scene but first try to see
        if the scene is not already published
        """
        if scene:
            logger.debug("Current state is:%s" % self.state)
            logger.debug("Publishing new state: %s" % scene)

            current_state = json.loads(self.state.message)
            new_state = json.loads(scene.message)

            if current_state['slug'] == new_state['slug']:
                logger.debug("Not publishing scene '%s' as it's "
                              "already published" % current_state['slug'])
            else:
                logger.debug("Publishing scene '%s' due to a callback "
                              "for new state" % new_state['slug'])
                self.director_scene_publisher.publish(scene)

            return True
        else:
            return False

    def current_uscs_message(self, *args, **kwargs):
        """
        Service run to tell nodes their state when they start up, this
        service should be depended on after director is up
        """
        if self.state:
            return self.state
        return USCSMessageResponse()

    def republish(self, *args, **kwargs):
        """
        Repeat last scene, without idempotentlity check
        """
        state = self.current_uscs_message()
        self.director_scene_publisher.publish(state)
        return EmptyResponse()

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
        with self.lock:
            logger.debug("Getting message {}".format(message))
            self.state = USCSMessageResponse()
            self.state.type = message.type
            self.state.message = message.message

    def _get_json_from_url(self, url, to_json=True):
        """
        Accepts a URL and returns json parsed version of it
        """
        response = urllib.request.urlopen(url)
        if response.code != 200:
            logger.error("Got non-200 error status (%s) from url: %s" % (response.code, url))
            rospy.sleep(3)
            return None

        try:
            message = response.read()
        except Exception:
            logger.error("Could not get response for initial state service")
            rospy.sleep(3)
            return None

        try:
            if to_json:
                message = json.loads(message)
            return message
        except ValueError:
            logger.error("invalid json: (%s)" % url)
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
            message = "Could not decode json message for msg_string: %s" % msg_string
            logger.error(message)
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
        except Exception as e:
            message = None
            logger.warning("Could not get scene for url: %s because %s" % (scene_url, e))
            logger.warning("sleeping for 3 seconds")
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

        logger.warning("Attempting to grab state from %s" % scene_url)
        message = self._get_json_from_url(scene_url)

        initial_state = USCSMessageResponse()
        initial_state.type = 'json'
        initial_state.message = json.dumps(message)

        return initial_state
