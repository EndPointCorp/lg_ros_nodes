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
from lg_common.helpers import message_is_nonzero
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

    - watched_topic e.g. "/spacenav/twist"
    - measurement - name of influxdb measurment - by default 'lg_stats'
    - msg_slot - dot-delimited list of attributes inside a message .e.g. 'header.seq' or 'angular.x'
    - debug_pub - publisher for publishing debugging Event.msg
    - resolution - how often we submit the data
    - how long to wait until old values get re-submitted
    - strategy: default, count, count_nonzero, average (either get attrib from message and write
     to influx, count messages or calculate average of slot values
    - influxdb_client - instance of influx client

    """
    def __init__(self,
                 watched_topic=None,
                 measurement=None,
                 msg_slot=None,
                 debug_pub=None,
                 resolution=5,
                 inactivity_resubmission=5,
                 strategy='default',
                 influxdb_client=None
                 ):
        if not msg_slot:
            msg = "Message slot not passed to Processor"
            rospy.logerr(msg)
            raise StatsConfigurationError(msg)
        if not watched_topic:
            msg = "Watched topic not passed to Processor"
            rospy.logerr(msg)
            raise StatsConfigurationError(msg)
        if not measurement:
            self.measurement = 'lg_stats'
        else:
            self.measurement = measurement

        self.watched_topic = watched_topic
        self.counter = 0
        self.messages = []
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
        rospy.loginfo("Initializing Processor instance: %s" % self)

    def __str__(self):
        return "<Processor instance for topic %s, msg_slot %s, strategy: %s>" % (self.watched_topic, self.msg_slot, self.strategy)

    def _start_resubmission_thread(self):
        """
        Starts two threads:
         - resubmission: responsible for re-submitting event data on topics which
          have low messaging traffic - useful for events liek mode changes or
          director messages
         - background: responsible for timely, frequent event submission of stats liek
          rate of touchscreen touch events or average value on a topic containing
          prox sensor range
        """

        if self.strategy == "default":
            rospy.loginfo("Starting 'default' strategy resubmission thread for %s" % self)
            self.resubmission_thread = threading.Thread(target=self._resubmission_thread)
            self.resubmission_thread.start()
        else:
            rospy.loginfo("Starting background thread for %s" % self)
            self._background_thread = threading.Thread(target=self._background_thread)
            self._background_thread.start()

    def _background_thread(self):
        """
        Flushes accumulated values
        """
        while not rospy.is_shutdown():
            rospy.logdebug("Background thread loop for %s" % self.watched_topic)
            self._wake_up_and_flush()
            rospy.logdebug("Background thread for %s going to sleep" % self.watched_topic)
            time.sleep(self.resolution)
        rospy.logdebug("Background thread finished for %s has finished" % self.watched_topic)

    def _resubmission_thread(self):
        """
        The method runs as a background thread.
        It checks whether there was any activity on the handled ROS
        topic every "inactivity_resubmission" period.
        If there was not, based on the last ROS message, a LG Stats
        update is sent out (both /lg_stats/debug topic as well as to InfluxDB.

        """
        while not rospy.is_shutdown():
            rospy.logdebug("Resubmission thread loop for %s" % self.watched_topic)
            self._resubmit_worker()
            rospy.logdebug("Resubmission thread sleeping ...")
            time.sleep(self.inactivity_resubmission)
        rospy.loginfo("Resubmission thread finished for %s has finished" % self.watched_topic)

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
                    rospy.logdebug("Re-submitting last message to InfluxDB ('%s') ..." % self.last_influx_data)

                    self.debug_pub.publish(self.last_out_msg)
                    # regenerate last message with new timestamp
                    regenerated_message = self.influxdb_client.get_data_for_influx(self.last_out_msg)
                    self.influxdb_client.write_stats(regenerated_message)
                    self.time_of_last_resubmission = time.time()
                else:
                    rospy.logdebug("The 'inactivity_resubmission' (%s) period has not "
                                   "elapsed yet = %s" % (self.inactivity_resubmission, elapsed))
            else:
                rospy.logdebug("Nothing received on topic %s so far." % self.watched_topic)

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
            rospy.logerr(msg)
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
                out_msg = Event(measurement=self.measurement,
                                src_topic=self.watched_topic,
                                field_name=self.msg_slot,
                                type="event",
                                value=str(value))
                return out_msg
            else:
                msg = "Could not get slot value for message '%s' using slot '%s'" % (src_msg, self.msg_slot)
                rospy.logerr(msg)

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
            self.counter += 1

            return False

        elif self.strategy == 'count_nonzero':
            """
            make each nonzero message increase a counter (where nonzero means that each
            slot or subslot needs to be different than 0)
            return False because no outbound message should be generated
            """
            is_nonzero = message_is_nonzero(src_msg)  # this may raise EmptyIncomingMessage

            if is_nonzero:
                self.counter += 1

            return False

    def _wake_up_and_flush(self):
        """
        """
        if self.strategy == 'default':
            """
            Do nothing
            """
            rospy.logdebug("Not flushing %s because of default strategy" % self)

        elif self.strategy == 'count':
            """
            Submit count, clean buffer
            """
            rospy.logdebug("Flushing %s" % self)
            out_msg = Event(measurement=self.measurement,
                            src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="rate",
                            value=str(self.counter))
            self.counter = 0
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self.submit_influxdata(out_msg)

        elif self.strategy == 'count_nonzero':
            """
            Submit count_nonzero, clean buffer
            """
            rospy.logdebug("Flushing %s" % self)
            out_msg = Event(measurement=self.measurement,
                            src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="nonzero_rate",
                            value=str(self.counter))
            self.counter = 0
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self.submit_influxdata(out_msg)

        elif self.strategy == 'average':
            """
            Calculate average, submit and clean buffer
            """
            rospy.logdebug("Flushing %s" % self)
            try:
                average = reduce(lambda x, y: float(x) + float(y), self.messages) / len(self.messages)
            except TypeError:
                """
                No messages to count
                """
                return
            self.messages = []
            out_msg = Event(measurement=self.measurement,
                            src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="average",
                            value=str(average))
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self.submit_influxdata(out_msg)
        else:
            """
            unknown strategy
            """
            self.logdebug("Unknown strategy %s" % self.strategy)
            pass

    def submit_influxdata(self, out_msg):
        """
        Accept out_msg of type Event and submit it to influxdb
        """

        influx_data = self.influxdb_client.get_data_for_influx(out_msg)
        out_msg.influx = str(influx_data)
        rospy.logdebug("Submitting to InfluxDB: '%s'" % influx_data)
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
            rospy.logerr(ex)
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
    inactivity_resubmission = rospy.get_param("~inactivity_resubmission", resolution)
    # source activities - returns list of dictionaries
    stats_sources = unpack_activity_sources(rospy.get_param("~activity_sources"))
    influxdb_client = get_influxdb_client()
    processors = []

    for stats_source in stats_sources:
        # for single stats_source dictionary please see unpack_activity_sources() docs
        # dynamic import based on package/message_class string representation
        msg_type = get_message_type_from_string(stats_source["message_type"])
        p = Processor(watched_topic=stats_source["topic"],
                      msg_slot=stats_source["slot"],
                      debug_pub=debug_topic_pub,
                      resolution=resolution,
                      strategy=stats_source["strategy"],
                      inactivity_resubmission=inactivity_resubmission,
                      influxdb_client=influxdb_client)
        p._start_resubmission_thread()  # keep it separated (easier testing)
        rospy.loginfo("Subscribing to topic '%s' (msg type: '%s') ..." % (stats_source["topic"], msg_type))
        rospy.Subscriber(stats_source["topic"], msg_type, p.process, queue_size=3)
        processors.append(p)

    # wake all processors that have strategy of average and count and make sure their buffers are emptied
    rospy.loginfo("Initializing lg_stats with: %s" % processors)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
