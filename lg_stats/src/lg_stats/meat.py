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


import time
import threading

import rospy

from lg_common.helpers import unpack_activity_sources
# from lg_common.helpers import write_log_to_file
from lg_common.helpers import get_message_type_from_string
from lg_common.helpers import get_nested_slot_value
from lg_common.helpers import SlotUnpackingException
from lg_stats.msg import Event
import submitters


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC_DEFAULT = "debug"


class EmptyIncomingMessage(Exception):
    pass


class StatsConfigurationError(Exception):
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
                 debug_pub=None,
                 resolution=20,
                 inactivity_resubmission=50,
                 strategy='default',
                 influxdb_client=None):
        if not msg_slot:
            msg = "Message slot not passed to Processor"
            rospy.logerr(msg)
            raise StatsConfigurationError(msg)
        if not watched_topic:
            msg = "Watchef topic not passed to Processor"
            rospy.logerr(msg)
            raise StatsConfigurationError(msg)

        self.watched_topic = watched_topic
        self.strategy = strategy
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

    def __str__(self):
        return "Processor instance for topic %s, msg_slot %s, strategy: %s" % (self.watched_topic, self.msg_slot, self.strategy)

    def _start_resubmission_thread(self):
        """
        TODO(wz) - remove thread spawn as it's not safe to use them inside ROS script
         instead use rospy.sleep() and rospy.Duration() or rospy.Rate()
        """

        if self.strategy == "default":
            rospy.loginfo("Starting resubmission thread for %s" % self)
            self.resubmission_thread = threading.Thread(target=self.resubmit)
            self.resubmission_thread.start()
        else:
            rospy.loginfo("Not starting resubmission thread for strategy: %s" % self.strategy)

    def _resubmit(self):
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

    def _resubmit_worker(self):
        """
        Thread that re-submits data influx
        """

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

    def _get_slot_value(self, msg):
        """
        Returns slot section of the message.
        If the slot section is empty, it evaluates to False (bool({})).
        If slot is not defined for this Processor, returns False.

        """
        if self.msg_slot:
            # this may be {} (empty message) and will evaluate still to False
            try:
                slot_value = get_nested_slot_value(self.msg_slot, msg)
                return slot_value[self.msg_slot]
            except SlotUnpackingException:
                return False
        else:
            msg = "Message slot not defined for topic %s" % (self.watched_topic)
            rospy.logerror(msg)
            raise StatsConfigurationError(msg)

    def _compare_messages(self, msg_1, msg_2):
        """
        Returns True if relevant field of the messages for
        this processor instance is equal.

        """
        if get_nested_slot_value(self.msg_slot, msg_1) == get_nested_slot_value(self.msg_slot, msg_2):
            return True
        else:
            return False

    def _get_outbound_message(self, src_msg):
        """
        Returns out-bound message for the /lg_stats/debug topic
        based on the data from the source topic message (src_msg).

        """
        if self.strategy == 'default':
            # self.resolution
            value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            if value:
                out_msg = Event(src_topic=self.watched_topic,
                                field_name=self.msg_slot,
                                type="event",
                                # value is always string, if source value is e.g.
                                # boolean, it's converted to a string here
                                value=value)
                return out_msg
            else:
                self.logerr("Could not get slot value for message %s" % src_msg)

        elif self.strategy == 'average':
            """
            calculate average - add value to list and wait for resubmission
            return False because no outbound message should be generated
            """
            value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            self.messages.append(value)

            return False

        elif self.strategy == 'count':
            """
            make each message increase a counter
            return False because no outbound message should be generated
            """
            value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            self.counter += 1

            return False

    def wake_up_and_flush(self, msg):
        """
        """
        if self.strategy == 'default':
            """
            Do nothing
            """
            pass

        elif self.strategy == 'count':
            """
            Submit count, clean buffer
            """
            out_msg = Event(src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="rate",
                            value=self.counter)

            self.submit_influxdata(out_msg)
            self.counter = 0

        elif self.strategy == 'average':
            """
            Calculate average, submit and clean buffer
            """
            self.messages = []
            average = reduce(lambda x, y: float(x) + float(y), self.messages) / len(self.messages)
            out_msg = Event(src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="average",
                            value=average)
            self.submit_influxdata(out_msg)

        else:
            """
            unknown strategy
            """
            self.loginfo("Unknown strategy %s" % self.strategy)
            pass

    def submit_influxdata(self, out_msg):
        """
        Accept out_msg of type Event and submit it to influxdb
        """

        influx_data = self.influxdb_client.get_data_for_influx(out_msg)
        out_msg.influx = str(influx_data)
        rospy.loginfo("Submitting to InfluxDB: '%s'" % influx_data)
        self.debug_pub.publish(out_msg)
        self.influxdb_client.write_stats(influx_data)

        return influx_data

    def process(self, msg):
        """
        Callback public method for messages flowing on observed topic

        """
        m = "Processor received: '%s'" % msg
        rospy.logdebug(m)

        try:
            out_msg = self._get_outbound_message(msg)

            if out_msg:
                influx_data = self.submit_influxdata(out_msg)
                """
                Do resubmission only for events
                """
                with self._lock:
                    self.last_in_msg = msg
                    self.time_of_last_in_msg = time.time()
                    self.last_influx_data = influx_data
                    self.last_out_msg = out_msg
                    self.time_of_last_resubmission = None

        except EmptyIncomingMessage, ex:
            rospy.logerror(ex)
            return


def get_influxdb_client():
    submitter_type = rospy.get_param("~submission_type", None)
    host = rospy.get_param("~host", None)
    port = rospy.get_param("~port", None)
    database = rospy.get_param("~database", None)
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
    resolution = rospy.get_param("~resolution", 2)
    inactivity_resubmission = rospy.get_param("~inactivity_resubmission", 20)
    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))
    influxdb_client = get_influxdb_client()
    processors = []

    for stats_source in stats_sources:
        # for single stats_source dictionary please see unpack_activity_sources() docs
        # dynamic import based on package/message_class string representation
        msg_type_module = get_message_type_from_string(stats_source["msg_type"])
        p = Processor(watched_topic=stats_source["topic"],
                      msg_slot=stats_source["slot"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution,
                      strategy=stats_source["strategy"],
                      inactivity_resubmission=inactivity_resubmission,
                      influxdb_client=influxdb_client)
        p._start_resubmission_thread()  # keep it separated (easier testing)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (stats_source["topic"], msg_type_module))
        rospy.Subscriber(stats_source["topic"], msg_type_module, p.process, queue_size=3)
        processors.append(p)

    # wake all procesors that have strategy of average and count and make sure their buffers are emptied
    for processor in processors:
        processor.wake_up_and_flush()
        rospy.sleep(resolution)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
