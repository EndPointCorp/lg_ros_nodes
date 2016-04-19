#!/usr/bin/env python
"""
NOTES:

- Session.msg
    borrowed from statistics/msg/Session.msg
    it's possible to introduce this dependency (or leave it as is)

- check statistics/StatsD.msg

Will need to define association between source topics and type of
    result statistical data (whether it's event based or time-series based).
    This will need to be either configured (not sure if "output-type" will
    be easy to blend with activity_sources processing. Or hard-coded in an
    internal relational matrix (output msg fields: type, application).

"""


import os
import json
import time
import threading

import rospy
from std_msgs.msg import String

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import get_params
from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_stats.msg import Event
import submitters


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC_DEFAULT = "debug"


class EmptyIncomingMessage(Exception):
    pass


class Processor(object):
    """
    Message processing is triggered by the reception of a next one.
    Didn't want any background threads or processes at this stage.
    It's lightweight anyway.
    The last (before ROS node shutdown) message is currently lost.

    """
    def __init__(self,
                 watched_topic=None,
                 msg_slot=None,
                 watched_field_name=None,
                 debug_pub=None,
                 resolution=20,
                 inactivity_resubmission=50,
                 influxdb_client=None):
        self.watched_topic = watched_topic
        self.watched_field_name = watched_field_name
        self.msg_slot = msg_slot
        self.debug_pub = debug_pub  # debug ROS publisher topic
        self.resolution = resolution  # seconds
        self.inactivity_resubmission = inactivity_resubmission  # seconds
        self.time_of_last_in_msg = None  # time of the last processed incoming message
        self.last_in_msg = None  # message from the source topic, last incoming mesage
        self.last_influx_data = None  # last message submitted to InfluxDB
        self.last_out_msg = None  # last /lg_stats/debug message
        self.time_of_last_resubmission = None
        self.influxdb_client = influxdb_client
        self.resubmission_thread = None
        self._lock = threading.Lock()

    def start_resubmission_thread(self):
        rospy.loginfo("Starting resubmission thread ...")
        self.resubmission_thread = threading.Thread(target=self.resubmit)
        self.resubmission_thread.start()

    def resubmit(self):
        """
        The method runs as a background thread.
        It checks whether there was any activity on the handled ROS
        topic every "inactivity_resubmission" period.
        If there was not, based on the last ROS message, a LG Stats
        update is sent out (both /lg_stats/debug topic as well as to InfluxDB.

        """
        while not rospy.is_shutdown():
            rospy.loginfo("Resubmission thread loop ...")
            self.resubmit_worker()
            rospy.loginfo("Resubmission thread sleeping ...")
            # another possibility is rate = rospy.Rate(1) ... rate.sleep() ... or rospy.sleep(2)
            time.sleep(2)
        rospy.loginfo("Resubmission thread finished.")

    def resubmit_worker(self):
        with self._lock:
            if self.last_in_msg and self.time_of_last_in_msg:
                if not self.time_of_last_resubmission:
                    self.time_of_last_resubmission = self.time_of_last_in_msg
                elapsed = time.time() - self.time_of_last_resubmission
                if int(round(elapsed)) > self.inactivity_resubmission:
                    rospy.loginfo("Resubmitting last message to InfluxDB ('%s') ..." %
                                  self.last_influx_data)
                    self.debug_pub.publish(self.last_out_msg)
                    self.influxdb_client.write_stats(self.last_influx_data)
                    self.time_of_last_resubmission = time.time()
                else:
                    rospy.loginfo("The 'inactivity_resubmission' (%s) period has not "
                                  "elapsed yet." % self.inactivity_resubmission)
            else:
                rospy.loginfo("Nothing received on this topic so far.")

    def get_whole_field_name(self):
        if self.msg_slot:
            return "%s.%s" % (self.msg_slot, self.watched_field_name)
        else:
            return self.watched_field_name

    def get_slot(self, msg):
        """
        Returns slot section of the message.
        If the slot section is empty, it evaluates to False (bool({})).
        If slot is not defined for this Processor, returns False.

        """
        if self.msg_slot:
            # this may be {} (empty message) and will evaluate still to False
            slot = json.loads(getattr(msg, self.msg_slot))
            if slot:
                return slot
            else:
                raise EmptyIncomingMessage("Empty slot section of the message.")
        else:
            return False

    def compare_messages(self, msg_1, msg_2):
        """
        Returns True if relevant field of the messages for
        this processor instance is equal.

        TODO:
        doesn't yet take into account slots

        """
        w = self.watched_field_name
        return True if getattr(msg_1, w) == getattr(msg_2, w) else False

    def get_outbound_message(self, src_msg):
        """
        Returns out-bound message for the /lg_stats/debug topic
        based on the data from the source topic message (src_msg).

        """
        slot = self.get_slot(src_msg)  # this may raise EmptyIncomingMessage
        if slot:
            value = str(slot[self.watched_field_name])
        else:
            value = str(getattr(src_msg, self.watched_field_name))
        out_msg = Event(src_topic=self.watched_topic,
                        field_name=self.get_whole_field_name(),
                        type="event",
                        # value is always string, if source value is e.g.
                        # boolean, it's converted to a string here
                        value=value)
        return out_msg

    def process(self, msg):
        """
        Processing of a message is in fact triggered by reception of the
        consecutive message. The current one is merely buffered.

        """
        m = "Processor received: '%s'" % msg
        rospy.loginfo(m)
        try:
            out_msg = self.get_outbound_message(msg)
        except EmptyIncomingMessage, ex:
            rospy.logwarn(ex)
            return
        influx_data = self.influxdb_client.get_data_for_influx(out_msg)
        out_msg.influx = str(influx_data)
        rospy.loginfo("Submitting to InfluxDB: '%s'" % influx_data)
        self.debug_pub.publish(out_msg)
        self.influxdb_client.write_stats(influx_data)
        with self._lock:
            self.last_in_msg = msg
            self.time_of_last_in_msg = time.time()
            self.last_influx_data = influx_data
            self.last_out_msg = out_msg
            self.time_of_last_resubmission = None


def get_influxdb_client():
    submitter_type = get_params("~submission_type", None)
    host = get_params("~host", None)
    port = get_params("~port", None)
    database = get_params("~database", None)
    if not submitter_type or not host or not port:
        raise RuntimeError("No InfluxDB connection details provided in the roslaunch configuration.")
    return getattr(submitters, submitter_type)(host=host, port=port, database=database)


def main():
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    debug_topic = "%s/%s" % (ROS_NODE_NAME, LG_STATS_DEBUG_TOPIC_DEFAULT)
    debug_topic_pub = rospy.Publisher(debug_topic, Event, queue_size=3)
    # resolution implementation is global across all watched topics which
    # may, may not be desirable ; in other words, we may want to have
    # the time resolution configurable per specified source topic processor instance
    resolution = get_params("~resolution", 2)
    inactivity_resubmission = get_params("~inactivity_resubmission", 20)
    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(get_params("~activity_sources"))
    influxdb_client = get_influxdb_client()
    processors = []
    for ss in stats_sources:
        # dynamic import based on package/message_class string representation
        msg_type_module = get_message_type_from_string(ss["msg_type"])
        p = Processor(watched_topic=ss["topic"],
                      msg_slot=ss["slot"],
                      watched_field_name=ss["strategy"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution,
                      inactivity_resubmission=inactivity_resubmission,
                      influxdb_client=influxdb_client)
        p.start_resubmission_thread()  # keep it separated (easier testing)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (ss["topic"], msg_type_module))
        rospy.Subscriber(ss["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
