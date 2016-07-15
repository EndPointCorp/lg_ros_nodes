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

import rospy
from std_msgs.msg import String


ROS_NODE_NAME = "lg_offliner"
LG_OFFLINER_DEBUG_TOPIC_DEFAULT = "debug"


class OfflinerException(Exception):
    pass


class ConnectivityResults(object):
    """
    Represents results of connectivity checks.

    """
    def __init__(self, max_length=100):
        """
        The data structure is a list of dictionaries.
            Each dictionary (list item) has check type as a key (e.g. icmp) and
            the value is return exit of the check command.

        max_length - maximum maintained length of the results list

        """
        self.data = list()
        self.max_length = max_length

    def add(self, item):
        if len(self.data) > self.max_length:
            # remove first item in the list
            _ = self.data.pop(0)
        self.data.append(item)

    def get_status(self, last_items_to_consider=2):
        """
        Return True, False based on last_items_to_consider evaluation.
        All checks (items of dictionaries) have the same weight.
        All results must be False in order to pronounce offline status.

        """
        for dict_check_results in self.data[-last_items_to_consider:]:
            for res in dict_check_results.values():
                if res:
                    return True
        else:
            return False


class Checker(object):
    def __init__(self,
                 check_every_seconds_delay=30,
                 check_types=None,
                 debug_topic_pub=None):
        self.check_every_seconds_delay = check_every_seconds_delay
        self.check_types = check_types
        self.debug_topic_pub = debug_topic_pub
        self._checker_thread = threading.Thread(target=self._checker_worker_thread)
        self._checker_thread.start()
        self._lock = threading.Lock()
        self._network_offline_flag = False
        self.log("Worker thread started.")

    def on_shutdown(self):
        self.log("Received shutdown request.")

    def log(self, msg):
        self.debug_topic_pub(msg)
        rospy.logdebug(msg)

    def _checker_worker_thread(self):
        while not rospy.is_shutdown():
            self.log("Background thread performing check loop ...")
            self._checker_worker()
            self.log("Background thread sleeping ...")
            for interval in range(0, self.check_every_seconds_delay):
                if rospy.is_shutdown():
                    break
                time.sleep(1)
        self.log("Thread finished.")

    def _checker_worker(self):
        # loop over all check_types commands and acquire results
        for check_type, cmd in self.check_types.items():
            proc = subprocess
        with self._lock:
            pass
            # just add results into the data structure

    def service(self):
        with self._lock:
            pass
            # just acquire data from the data structure and emit
            # call evaluate

    def __str__(self):
        return "%s: performing checks: '%s'" % (self.__class__.__name__,
                                                self.check_types)


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, String, queue_size=10)
    check_every_seconds_delay = rospy.get_param("~check_every_seconds_delay")
    # this returns a string representation of a dictionary
    checks_str = rospy.get_param("~checks")
    check_types = ast.literal_eval(checks_str)
    rospy.loginfo("Configured checks to run: '%s'" % check_types)
    # TODO
    # implement delay configuration
    # TODO
    # source activities - returns list of dictionaries
    # use activity config configuration style here?
    # like e.g. dns:host which to check ... possible to use this config format here?
    checker = Checker(check_every_seconds_delay=check_every_seconds_delay,
                      check_types=check_types,
                      debug_topic_pub=debug_topic_pub)
    rospy.on_shutdown(checker.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
