#!/usr/bin/env python
"""
lg_offliner ROS node implementation.

Check network connectivity and inform listeners.

DEBUG topic - publish status, check results as they are performed


"""


import time

import rospy
from std_msgs.msg import String


ROS_NODE_NAME = "lg_offliner"
LG_OFFLINER_DEBUG_TOPIC_DEFAULT = "debug"


class OfflinerException(Exception):
    pass


class CheckerWorker(object):
    """
    This class has methods of the names corresponding to this
    ros node configuration.

    Methods are called from the main Checker instance.

    """
    def __init__(self):
        pass


class Checker(object):
    def __init__(self):
        self.worker = CheckerWorker()
        # TODO
        # publish on the debug topic that is up and running
        #self.debug_pub.publish(self.last_out_msg)

    def on_shutdown(self):
        rospy.loginfo("Received shutdown request.")
        # TODO
        # publish on the debug topic that it's going down
        #self.debug_pub.publish(self.last_out_msg)

    def process(self):
        # TODO
        # will be run every configuration period and do stuff
        # period configurable
        while True:
            time.sleep(5)

    def __str__(self):
        return "good self description"


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_OFFLINER_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, String, queue_size=10)
    # TODO
    # implement delay configuration
    # TODO
    # source activities - returns list of dictionaries
    # use activity config configuration style here?
    # like e.g. dns:host which to check ... possible to use this config format here?
    checker = Checker()
    rospy.on_shutdown(checker.on_shutdown)
    rospy.loginfo("Started, spinning %s ..." % ROS_NODE_NAME)
    rospy.spin()
