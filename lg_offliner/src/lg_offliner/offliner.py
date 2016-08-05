#!/usr/bin/env python
"""
lg_offliner ROS node implementation.

Check network connectivity and inform listeners.

DEBUG topic - publish status, check results as they are performed

"""


import ast
import time
import threading
import subprocess
import pprint

import rospy
from std_msgs.msg import String, Bool

from lg_common import helpers
from lg_offliner.srv import Offline


ROS_NODE_NAME = "lg_offliner"
LG_OFFLINER_DEBUG_TOPIC_DEFAULT = "debug"
LG_OFFLINER_OFFLINE_TOPIC_DEFAULT = "offline"


class OfflinerException(Exception):
    pass


class ConnectivityResults(object):
    """
    Represents results of connectivity checks.

    """
    def __init__(self, max_length=100):
        """
        The data structure is a list of dictionaries.
            Each dictionary keys are connectivity check commands
            and values are the commands' results.

        max_length - maximum maintained length of the results list

        """
        self.data = list()
        self.max_length = max_length

    def add(self, item):
        if len(self.data) > self.max_length:
            # remove first item in the list
            _ = self.data.pop(0)
        self.data.append(item)

    def am_i_offline(self, last_items_to_consider=2):
        """
        Return True, False based on last_items_to_consider evaluation.
        All checks (items of dictionaries) have the same weight.
        All results must be non-zero in order to pronounce offline status.

        """
        for dict_check_results in self.data[-last_items_to_consider:]:
            for res in dict_check_results.values():
                if res == 0:
                    return False
        else:
            return True


class Checker(object):
    """
    Connectivity checker class.

    check_cmds - list of Unix utils commands to run whose result determines
        whether we are internet online or offline.
    debug_topic_pub - ROS debug topic to publish info about current activities

    """
    def __init__(self,
                 check_every_seconds_delay=30,
                 check_cmds=None,
                 debug_topic_pub=None,
                 offline_topic_pub=None):
        self.check_every_seconds_delay = check_every_seconds_delay
        self.check_cmds = check_cmds
        self._current_offline_status = False
        self.debug_topic_pub = debug_topic_pub
        self.offline_topic_pub = offline_topic_pub
        self._checker_thread = threading.Thread(target=self._checker_worker_thread)
        self._checker_thread.start()
        self._lock = threading.Lock()
        self._results = ConnectivityResults()  # lock access here
        self.service = rospy.Service("%s/status" % ROS_NODE_NAME,
                                     Offline,
                                     self.get_offline_status)
        self.log("Worker thread started.")

    def on_shutdown(self):
        self.log("Received shutdown request.")

    def log(self, msg):
        rospy.logdebug(msg)
        self.debug_topic_pub.publish(msg)

    def _checker_worker_thread(self):
        while not rospy.is_shutdown():
            self.log("Background thread performing check loop ...")
            self._checker_worker()
            self.evaluate_current_status()
            self.log("Background thread sleeping ...")
            for interval in range(0, self.check_every_seconds_delay):
                if rospy.is_shutdown():
                    self.log("Shutdown received, thread finishing ...")
                    break
                time.sleep(1)
        self.log("Thread finished.")

    def _checker_worker(self):
        # loop over all check_cmds commands and acquire results
        results = {}
        for cmd in self.check_cmds:
            res = subprocess.call(cmd.split())
            self.log("'%s' finished, result: %s" % (cmd, res))
            results[cmd] = res
            if rospy.is_shutdown():
                return
        with self._lock:
            # just add results into the data structure
            self._results.add(results)

    def get_offline_status(self, _):
        with self._lock:
            status = self._results.am_i_offline()
        self.log("Am I offline: %s" % status)
        return status

    def evaluate_current_status(self):
        with self._lock:
            flag = self._results.am_i_offline()
        self.log("Am I offline: %s" % flag)
        if flag != self._current_offline_status:
            self._current_offline_status = flag
            if self._current_offline_status:
                self.on_becoming_offline()
            else:
                self.on_becoming_online()

    def on_becoming_online(self):
        self.offline_topic_pub.publish(False)
        # TODO
        # send the send_on_online message

    def on_becoming_offline(self):
        self.offline_topic_pub.publish(True)
        # TODO
        # send the send_on_offline message

    def __str__(self):
        return "%s: performing checks: '%s'" % (self.__class__.__name__,
                                                self.check_cmds)


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_DEBUG_TOPIC_DEFAULT)
    offline_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_OFFLINE_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, String, queue_size=10)
    offline_topic_pub = rospy.Publisher(offline_topic, Bool, queue_size=10)
    check_every_seconds_delay = rospy.get_param("~check_every_seconds_delay")
    # this returns a string representation of a list
    checks_str = rospy.get_param("~checks")
    check_cmds = ast.literal_eval(checks_str)
    rospy.loginfo("Configured to run following check commands:\n%s" % pprint.pformat(check_cmds))
    # TODO
    # need to extend lg_common.helpers.unpack_activity_sources to read single value properly
    # currently it only supports min_value, max_value
    #send_on_online = rospy.get_param("~send_on_online")
    #rospy.loginfo(send_on_online)
    #ss = helpers.unpack_activity_sources(send_on_online)
    #rospy.loginfo(ss)
    checker = Checker(check_every_seconds_delay=check_every_seconds_delay,
                      check_cmds=check_cmds,
                      debug_topic_pub=debug_topic_pub,
                      offline_topic_pub=offline_topic_pub)
    rospy.on_shutdown(checker.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
