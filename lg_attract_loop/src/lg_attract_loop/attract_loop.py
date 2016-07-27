#!/usr/bin/env python

import os
import json
import rospy
import requests

from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage
from rosapi import params


class DirectorAPIProxy:
    def __init__(self, director_api_url):
        """
        Class responsible for getting content from director api
        """
        rospy.loginfo("Initializing Attract Loooooooooooop")
        self.director_api_url = director_api_url
        rospy.loginfo("Using director API url: %s" % self.director_api_url)

    def get(self, uri):
        try:
            url = "%s%s" % (self.director_api_url, uri)
            return requests.get(url).content
        except Exception:
            rospy.logerr("Could not get content from URL: %s" % url)


def MockFunc(*args, **kwargs):
    pass


class AttractLoop:
    def __init__(self, api_proxy, director_scene_publisher,
                 director_presentation_publisher, stop_action,
                 earth_query_publisher, earth_planet_publisher,
                 default_presentation=None, default_planet='earth',
                 set_earth=MockFunc):
        """
        Class responsible for playing back presentations/scenes that are marked as "attract_loop"
        in Liquid Galaxy content management system.

        """
        self.api_proxy = api_proxy
        self.earth_query_publisher = earth_query_publisher
        self.stop_action = stop_action
        self.default_presentation = default_presentation
        self.default_planet = default_planet
        self.earth_planet_publisher = earth_planet_publisher
        self.director_scene_publisher = director_scene_publisher
        self.director_presentation_publisher = director_presentation_publisher
        self.attract_loop_queue = []
        self.play_loop = False
        self.scene_timer = 0
        self.set_earth = set_earth
        self.initialize_timer()

    def initialize_timer(self):
        """
        Each scene has a timer - countdown is using 1s resolution.
        """
        rospy.Timer(rospy.Duration(1), self._play_attract_loop)

    def _process_activity_state_change(self, message):
        """
        when message (that comes from /activity/active) equals True
        then we begin the playback
        otherwise we stop
        """
        self.scene_timer = 0

        if message.data is True and self.play_loop is True:
            rospy.loginfo("Director: Attract loop becoming inactive")
            self.play_loop = False
            self._stop_attract_loop()
        elif message.data is False and self.play_loop is False:
            self._switch_to_planet()
            rospy.loginfo("Director: Attract loop becoming active")
            self.play_loop = True
            rospy.sleep(2)
        else:
            rospy.logwarn("Activity message contained state %s and current state is %s - that's weird" % (message.data, self.play_loop))

    def _stop_attract_loop(self):
        """
        Emits ROS message on the basis of pre-configured action.
        This action is executed when system becomes active.
        """
        self.play_loop = False

        rospy.loginfo("Stopping scene timer")

        if self.stop_action == 'stop_playtour':
            self._stop_playtour()
        elif self.stop_action == 'go_blank':
            self._stop_playtour()
            self._publish_blank_scene()
        elif self.stop_action == 'go_blank_and_switch_to_planet':
            self._stop_playtour()
            self._publish_blank_scene()
            self._switch_to_planet()
        elif self.stop_action == 'load_presentation':
            self._stop_playtour()
            if self.default_presentation:
                pass
            else:
                rospy.logerr("No default presentation defined")
        else:
            pass

    def _switch_to_planet(self):
        """
        Emits a message with planet change taken from configuration
        """
        switch_to_planet_msg = String(data=self.default_planet)
        rospy.loginfo("Executing 'switch_to_planet' action")
        self.earth_planet_publisher.publish(switch_to_planet_msg)

    def _stop_playtour(self):
        """
        Emits empty message on /earth/query/tour to stop the tour

        **PUFF MAGIC**

        """
        stop_tour_msg = String(data='')
        rospy.loginfo("Executing 'stop_playtour' action")
        self.earth_query_publisher.publish(stop_tour_msg)

    def _publish_blank_scene(self):
        """
        Emits a scene with empty windows to clean up all assets from screens
        """
        rospy.loginfo("Playing blank scene")

        viewports = [viewport.split('/')[2] for viewport in params.get_param_names() if '/viewport/' in viewport]

        scene = {
            "description": "attract loop blank scene",
            "duration": 666,
            "name": "attract loop blank scene",
            "resource_uri": "no uri",
            "slug": "attract-loop-break",
            "windows": []
        }

        for viewport_name in viewports:
            window = {
                "assets": [],
                "y_coord": 666,
                "x_coord": 666,
                "height": 666,
                "width": 666,
                "activity": "no_activity",
                "presentation_viewport": viewport_name
            }
            scene['windows'].append(window)

        scene_msg = GenericMessage(type='json', message=json.dumps(scene))
        self.director_scene_publisher.publish(scene_msg)

    def _play_attract_loop(self, event):
        """
        Play items from the queue
        If there's no queue - populate it
        Every item in the queue is a scene + presentation
        """
        rospy.logdebug("Populating attract loop queue with content")
        try:
            if self.attract_loop_queue:
                rospy.logdebug("Attract_loop_queue alrady contains content (%s) continuing from last played scene" % self.attract_loop_queue)
            else:
                self.attract_loop_queue = self._fetch_attract_loop_content()
                rospy.logdebug("Populated attract_loop_queue with %s" % self.attract_loop_queue)
            self._play_attract_loop_item()
        except Exception, e:
            rospy.loginfo("Failed to populate attract loop queue with content because %s - sleeping for 60 seconds" % e)
            rospy.sleep(60)

    def _play_scene(self, lazy_scene, lazy_presentation):
        """
        Accepts lazy scene object and lazy presentation object
        Fetches full, non-lazy versions of objects
        Emits a /director/scene and /director/presentation message
        """
        full_presentation = self._fetch_presentation_by_slug(lazy_presentation['slug'])
        full_scene = self._fetch_by_resource_uri(lazy_scene['resource_uri'])
        duration = full_scene['duration']
        rospy.logdebug("Playing scene %s from presentation %s with duration %s" % (full_scene, full_presentation, duration))

        scene_msg = GenericMessage(type='json', message=json.dumps(full_scene))
        presentation_msg = GenericMessage(type='json', message=json.dumps(full_presentation))

        self.director_scene_publisher.publish(scene_msg)
        self.director_presentation_publisher.publish(presentation_msg)

        self.scene_timer = duration

    def _play_attract_loop_item(self):
        """
        plays back a scene by publishing it to /director/scene and
        its presentation to /director/presentation
        handles the timer countdown
        """
        rospy.logdebug("Executing _play_attract_loop_item - self.play_loop=%s" % self.play_loop)
        rospy.logdebug("Scene timer=%s" % self.scene_timer)

        if self.play_loop and self.scene_timer <= 0:
            rospy.logdebug("Inside play loop - queue size=%s" % (len(self.attract_loop_queue)))
            playback_item = self.attract_loop_queue[0]
            if playback_item['scenes']:  # play item back or remove it from queue if no scenes
                lazy_presentation = playback_item['presentation']
                lazy_scene = playback_item['scenes'].pop(0)  # take it away - forever
                self._play_scene(lazy_scene, lazy_presentation)
                rospy.logdebug("Item to played back taken from self.attract_loop_queue: %s" % playback_item)
            else:
                rospy.logdebug("Removing item as it does not longer have any scenes inside it")
                self.attract_loop_queue.remove(playback_item)

        self.scene_timer -= 1

    def _fetch_attract_loop_content(self):
        """
        Populate attract loop content so it looks like this:
        [
          { "presentation": {
             "description": "Catlin Seaview Survey",
             "name": "Catlin Seaview Surveys",
                 "slug": "catlin-seaview-surveys"
         },
             "scenes": [
        {
          "description": "Gaudi Cathedral",
          "duration": 1800,
          "name": "Gaudi Cathedral",
          "resource_uri": "/director_api/scene/gaudi-cathedral/",
          "slug": "gaudi-cathedral",
          "touchscreen_visible": true
        }
             ]
          }
         ]


        The "presentation" object comes from `http://lg-head:8088/director_api/presentationgroup/<presentationgroup>` presentations
        attribute.
        The "scenes" attribute is a list of scenes for each presentation. All object are lazy here.
        """
        # fetch presentationgroups
        content = []
        presentationgroups = self._fetch_attract_loop_presentationgroups()
        if presentationgroups:
            rospy.loginfo("Fetched %s presentationgroups" % len(presentationgroups))
            presentations = self._fetch_presentationgroup_presentations(presentationgroups)
            if presentations:
                rospy.loginfo("Fetched %s presentations" % len(presentations))
                rospy.logdebug("Here they are: %s" % presentations)
                for presentation in presentations:
                    rospy.logdebug("Preparing content object")
                    presentation_object = {"presentation": presentation,
                                           "scenes": self._fetch_presentation_by_slug(presentation['slug'])['scenes']}
                    rospy.logdebug("Appending presentation object %s to fetched content" % presentation_object)
                    rospy.loginfo("Fetched %s scenes" % len(presentation_object['scenes']))
                    content.append(presentation_object)
            rospy.logdebug("Fetched new content: %s" % content)
        else:
            rospy.logdebug("No presentation groups found in attract loop sleeping for 120 seconds")
            rospy.sleep(120)

        return content

    def _fetch_presentation_by_slug(self, presentation_name):
        """
        Accepts slug of presentation and fetches full object of presentation
        """
        full_presentation = json.loads(self.api_proxy.get("/director_api/presentation/%s/" % presentation_name))
        return full_presentation

    def _fetch_by_resource_uri(self, resource_uri):
        """
        Return json serialzed response from resource_uri url
        """
        fetched_object = json.loads(self.api_proxy.get("%s?format=json" % resource_uri))
        return fetched_object

    def _fetch_presentationgroup_presentations(self, presentationgroups):
        """
        """
        attract_loop_presentations = []
        try:
            for presentation in presentationgroups:
                presentation_resource_uri = presentation['resource_uri']
                presentations_request = self.api_proxy.get("%s" % (presentation_resource_uri))
                presentations = json.loads(presentations_request)['presentations']
                attract_loop_presentations.extend(presentations)
            return attract_loop_presentations
        except Exception, e:
            rospy.logerr("Could not fetch presentations from presentationgroups (%s) because %s" % (presentationgroups, e))
            return []

    def _fetch_attract_loop_presentationgroups(self):
        try:
            presentationgroup_request = self.api_proxy.get("/director_api/presentationgroup/?attract_loop=True")
            presentationgroups = json.loads(presentationgroup_request)['objects']
            assert(type(presentationgroups) == list), "Presentationgroups type is not list"
            return presentationgroups
        except Exception, e:
            rospy.logerr("Could not get presentationgroups because: %s - sleeping for 10 seconds" % e)
            rospy.sleep(10)
            return []
