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
                 debug_topic_pub=None):
        self.check_every_seconds_delay = check_every_seconds_delay
        self.check_cmds = check_cmds
        self.debug_topic_pub = debug_topic_pub
        self._checker_thread = threading.Thread(target=self._checker_worker_thread)
        self._checker_thread.start()
        self._lock = threading.Lock()
        self._results = ConnectivityResults()  # lock access here
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
            with self._lock:
                flag = self._results.am_i_offline()
            self.log("Am I offline: %s" % flag)
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
        with self._lock:
            # just add results into the data structure
            self._results.add(results)

    def service(self):
        with self._lock:
            status = self._results.am_i_offline()
        self.log("Am I offline: %s" % status)
        # TODO
        # now emit data

    def __str__(self):
        return "%s: performing checks: '%s'" % (self.__class__.__name__,
                                                self.check_cmds)


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, String, queue_size=10)
    check_every_seconds_delay = rospy.get_param("~check_every_seconds_delay")
    # this returns a string representation of a dictionary
    checks_str = rospy.get_param("~checks")
    check_cmds = ast.literal_eval(checks_str)
    rospy.loginfo("Configured to run following check commands:\n%s" % pprint.pformat(check_cmds))
    # TODO
    # configure the output offine message / offline activity trigger
    # use activity config configuration style here?
    checker = Checker(check_every_seconds_delay=check_every_seconds_delay,
                      check_cmds=check_cmds,
                      debug_topic_pub=debug_topic_pub)
    rospy.on_shutdown(checker.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
