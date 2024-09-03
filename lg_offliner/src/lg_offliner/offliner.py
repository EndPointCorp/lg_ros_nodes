#!/usr/bin/env python3
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
import socket

import rospy
from std_msgs.msg import String, Bool

from lg_common import helpers
from lg_msg_defs.srv import Offline


ROS_NODE_NAME = "lg_offliner"
from lg_common.logger import get_logger
logger = get_logger(ROS_NODE_NAME)
LG_OFFLINER_DEBUG_TOPIC_DEFAULT = "debug"
LG_OFFLINER_OFFLINE_TOPIC_DEFAULT = "offline"


class ConnectivityResults(object):
    """
    Represents results of the connectivity checks.

    """
    def __init__(self, max_num_of_rounds_to_retain=100, num_of_last_check_rounds_consider=2):
        """
        The data structure is a list of dictionaries.
            Each dictionary keys are connectivity check commands
            and values are the commands' results.

        max_num_of_rounds_to_retain - maximum maintained length of the results list

        """
        self.data = list()
        self.max_num_of_rounds_to_retain = max_num_of_rounds_to_retain
        self.num_of_last_check_rounds_consider = num_of_last_check_rounds_consider

    def add(self, item):
        if len(self.data) == self.max_num_of_rounds_to_retain:
            # remove first item in the list
            _ = self.data.pop(0)
        self.data.append(item)

    def am_i_offline(self):
        """
        Return True, False based on num_of_last_check_rounds_consider
            rounds evaluations.
        All checks (items of dictionaries) have the same weight.
        All results must be non-zero in order to pronounce offline status.
            if any results in the considered rounds is 0, the
            online status is still maintained.

        """
        # -num_of_last_check_rounds_consider won't raise IndexError when len(self.data) is smaller
        logger.debug("called am_i_offline and data is: %s" % self.data)
        if not self.data:
            return False
        for dict_check_results in self.data[-self.num_of_last_check_rounds_consider:]:
            for res in list(dict_check_results.values()):
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

    online_pubs, offline_pubs - both are lists of dictionaries, the dictionaries
        contain ROS publisher and a predefined (from the configuration) messages
        to be send via the publisher on becoming online and offline.

    """
    def __init__(self,
                 check_every_seconds_delay=30,
                 max_num_of_rounds_to_retain=100,
                 num_of_last_check_rounds_consider=2,
                 check_cmds=None,
                 debug_topic_pub=None,
                 offline_topic_pub=None,
                 online_pubs=None,
                 offline_pubs=None):
        self.check_every_seconds_delay = check_every_seconds_delay
        self.check_cmds = check_cmds
        self._current_offline_status = False
        self.debug_topic_pub = debug_topic_pub
        self.offline_topic_pub = offline_topic_pub
        self.online_pubs = online_pubs
        self.offline_pubs = offline_pubs
        self._checker_thread = threading.Thread(target=self._checker_worker_thread)
        self._checker_thread.start()
        self._lock = threading.Lock()
        # lock access here for ConnectivityResults
        self._results = ConnectivityResults(
            max_num_of_rounds_to_retain=max_num_of_rounds_to_retain,
            num_of_last_check_rounds_consider=num_of_last_check_rounds_consider)
        self.service = rospy.Service("%s/status" % ROS_NODE_NAME,
                                     Offline,
                                     self.get_offline_status)
        self.log("Worker thread started.")

    def on_shutdown(self):
        self.log("Received shutdown request.")

    def log(self, msg):
        logger.debug(msg)
        self.debug_topic_pub.publish(msg)

    def do_active_delay(self):
        for interval in range(0, self.check_every_seconds_delay):
            if rospy.is_shutdown():
                self.log("Shutdown received, delay routine interrupted.")
                break
            time.sleep(1)

    def _checker_worker_thread(self):
        self.do_active_delay()
        while not rospy.is_shutdown():
            self.log("Background thread performing check loop ...")
            self._checker_worker()
            self._evaluate_current_status()
            self.log("Background thread sleeping ...")
            self.do_active_delay()
        self.log("Thread finished.")

    def _checker_worker(self):
        """
        Loop over all check_cmds commands and acquire results.

        """
        results = {}
        for cmd in self.check_cmds:
            res = subprocess.call(cmd.split(), stdout=open('/dev/null', 'w'))
            self.log("'%s' finished, result: %s" % (cmd, res))
            results[cmd] = res
            if rospy.is_shutdown():
                return
        with self._lock:
            # just add results into the data structure
            self._results.add(results)

    def get_offline_status(self, _):
        """
        ROS service method.

        """
        with self._lock:
            status = self._results.am_i_offline()
        self.log("Am I offline: %s" % status)
        return status

    def _evaluate_current_status(self):
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
        for pub in self.online_pubs:
            pub["publisher"].publish(pub["message"])

    def on_becoming_offline(self):
        self.offline_topic_pub.publish(True)
        for pub in self.offline_pubs:
            pub["publisher"].publish(pub["message"])

    def __str__(self):
        return "%s: performing checks: '%s'" % (self.__class__.__name__,
                                                self.check_cmds)


def process_custom_publishers(send_on_online=None, send_on_offline=None):
    """
    Reads corresponding ROS params.
    Each is a list of dicts, each dict as returned by unpack_activity_sources.
    The output is a dictionary consisting of ROS publisher and predefined message instance.

    Note:
        messages with subslots (such as GenericMessage message.slug) will need more care
        in processing in this function, not yet implemented.

    """
    def process(senders):
        pub_senders = []
        for sender in senders:
            msg_type = helpers.get_message_type_from_string(sender["message_type"])
            publisher = rospy.Publisher(sender["topic"], msg_type, queue_size=5)
            msg = msg_type()
            setattr(msg, sender["slot"], sender["value"])
            pub_senders.append(dict(publisher=publisher, message=msg))
        return pub_senders

    online_pubs = process(send_on_online)
    offline_pubs = process(send_on_offline)
    return online_pubs, offline_pubs


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_DEBUG_TOPIC_DEFAULT)
    offline_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_OFFLINE_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, String, queue_size=5)
    offline_topic_pub = rospy.Publisher(offline_topic, Bool, queue_size=5)
    check_every_seconds_delay = rospy.get_param("~check_every_seconds_delay")
    max_num_of_rounds_to_retain = rospy.get_param("~max_num_of_rounds_to_retain", 100)
    num_of_last_check_rounds_consider = rospy.get_param("~num_of_last_check_rounds_consider", 2)
    socket_timeout = rospy.get_param("~socket_timeout", 1)
    socket.setdefaulttimeout(socket_timeout)
    checks_str = rospy.get_param("~checks")  # this returns a string representation of a list
    check_cmds = ast.literal_eval(checks_str)
    logger.info("Configured to run following check commands:\n%s" % pprint.pformat(check_cmds))
    send_on_online_str = rospy.get_param("~send_on_online")
    logger.info(send_on_online_str)
    send_on_online = helpers.unpack_activity_sources(send_on_online_str)
    logger.info("send_on_online: %s" % send_on_online)
    send_on_offline_str = rospy.get_param("~send_on_offline")
    logger.info(send_on_offline_str)
    send_on_offline = helpers.unpack_activity_sources(send_on_offline_str)
    logger.info("send_on_offline: %s" % send_on_offline)
    online_pubs, offline_pubs = process_custom_publishers(send_on_online=send_on_online,
                                                          send_on_offline=send_on_offline)
    checker = Checker(check_every_seconds_delay=check_every_seconds_delay,
                      max_num_of_rounds_to_retain=max_num_of_rounds_to_retain,
                      num_of_last_check_rounds_consider=num_of_last_check_rounds_consider,
                      check_cmds=check_cmds,
                      debug_topic_pub=debug_topic_pub,
                      offline_topic_pub=offline_topic_pub,
                      online_pubs=online_pubs,
                      offline_pubs=offline_pubs)
    rospy.on_shutdown(checker.on_shutdown)
    logger.info("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
