#!/usr/bin/env python3
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
from lg_common.helpers import get_message_type_from_string
from lg_common.helpers import get_nested_slot_value
from lg_common.helpers import message_is_nonzero
from lg_common.helpers import SlotUnpackingException
from lg_common.helpers import get_random_string
from lg_stats.msg import Event
from . import submitters
from functools import reduce


ROS_NODE_NAME = "lg_stats"
LG_STATS_DEBUG_TOPIC_DEFAULT = "debug"


class EmptyIncomingMessage(Exception):
    pass


class StatsConfigurationError(Exception):
    pass


class Processor(object):
    """

    tl;dr Processor watches a given ROS topic. It knows how to discard
    or observe specific slots (sub-slots) of ROS messages. It can calculate an average of sum
    of ROS message values or submit *every* message as event (default strategy).

    - watched_topic e.g. "/spacenav/twist"
    - measurement - name of influxdb measurment - by default 'lg_stats'
    - msg_slot - dot-delimited list of attributes inside a message .e.g. 'header.seq' or 'angular.x'
    - debug_pub - publisher for publishing debugging Event.msg
    - resolution - how often we submit the data
    - how long to wait until old values get re-submitted
    - strategy: default_session, default, count, count_nonzero, average (either get attrib from message and write
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
            if strategy == 'default' or strategy == 'default_session':
                self.measurement = 'lg_stats_event'
            else:
                self.measurement = 'lg_stats_metric'
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

    def __repr__(self):
        return self.__str__()

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

        if self.strategy == "default" or self.strategy == 'default_session':
            rospy.loginfo("Starting %s strategy resubmission thread for %s" % (self.strategy, self))
            self.resubmission_thread = threading.Thread(target=self._resubmission_thread)
            self.resubmission_thread.start()
        else:
            rospy.loginfo("Starting background thread for %s" % self)
            self._periodic_flush_thread = threading.Thread(target=self._periodic_flush_thread)
            self._periodic_flush_thread.start()

    def on_shutdown(self):
        rospy.loginfo("Received shutdown for periodic/resubmission")

    def _periodic_flush_thread(self):
        """
        Flushes accumulated values for non-default strategies
        """
        while not rospy.is_shutdown():
            rospy.logdebug("Background thread loop for %s" % self.watched_topic)
            self._flushing_worker()
            rospy.logdebug("Background thread for %s going to sleep" % self.watched_topic)
            for interval in range(0, self.resolution):
                if rospy.is_shutdown():
                    break
                rospy.sleep(1)

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
            for interval in range(0, self.resolution):
                if rospy.is_shutdown():
                    break
                rospy.sleep(1)
        rospy.loginfo("Resubmission thread finished for %s has finished" % self.watched_topic)

    def _resubmit_worker(self):
        """
        Thread that re-submits data influx.
        It modifies the timestamp to current time as well as the
        value which needs to indicate that we're re-submitting a message
        as opposed to submission of a new one.
        """

        with self._lock:
            if self.last_in_msg and self.time_of_last_in_msg:
                if not self.time_of_last_resubmission:
                    self.time_of_last_resubmission = self.time_of_last_in_msg
                elapsed = time.time() - self.time_of_last_resubmission
                if int(round(elapsed)) > self.inactivity_resubmission:
                    rospy.logdebug("Re-submitting last message to InfluxDB ('%s') ..." % self.last_influx_data)

                    # regenerate last message with new timestamp and diminished value
                    self.last_out_msg.value = "0.5"
                    self.last_out_msg.span = str(self.resolution)
                    self.debug_pub.publish(self.last_out_msg)
                    regenerated_message = self.influxdb_client.get_data_for_influx(self.last_out_msg, self.measurement)
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
                rospy.logerr("Could not get slot value for message %s with slot %s" % (msg, self.msg_slot))
                return ''
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
        Accepts incoming ROS message that came on a topic that Processor listens at.

        Returns outbound message for the /lg_stats/debug topic
        based on the data from the source topic message (src_msg).

        This message will contain `influx` attribute that is ready for influxdb submission.

        """
        if self.strategy == 'default':
            # self.resolution
            slot_value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            out_msg = Event(measurement=self.measurement,
                            src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="event",
                            metadata=str(slot_value),
                            span=str(self.resolution),
                            value="1.0")
            return out_msg

        elif self.strategy == 'default_session':
            self.session_id = get_random_string(N=8, uppercase=False)
            slot_value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            out_msg = Event(measurement=self.measurement,
                            src_topic="session:" + self.watched_topic,
                            field_name=self.msg_slot,
                            type="event",
                            metadata=str(slot_value) + "__%s" % self.session_id,
                            span=str(self.resolution),
                            value="1.0")
            return out_msg

        elif self.strategy == 'average':
            """
            calculate average - add value to list and wait for resubmission
            return False because no outbound message should be generated
            """
            slot_value = self._get_slot_value(src_msg)  # this may raise EmptyIncomingMessage
            self.messages.append(slot_value)

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

    def _flushing_worker(self):
        """
        Flushes non-default strategy buffers - calculates rates/counts etc
        """
        if self.strategy == 'default' or self.strategy == 'default_session':
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
                            metadata="flush",
                            span=str(self.resolution),
                            value=str(self.counter))
            self.counter = 0
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self._submit_influxdata(out_msg)

        elif self.strategy == 'count_nonzero':
            """
            Submit count_nonzero, clean buffer
            """
            rospy.logdebug("Flushing %s" % self)
            out_msg = Event(measurement=self.measurement,
                            src_topic=self.watched_topic,
                            field_name=self.msg_slot,
                            type="nonzero_rate",
                            metadata="flush",
                            span=str(self.resolution),
                            value=str(self.counter))
            self.counter = 0
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self._submit_influxdata(out_msg)

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
                            metadata="flush",
                            span=str(self.resolution),
                            value=str(average))
            rospy.logdebug("Flushing %s with out_msg=%s" % (self, out_msg))
            self._submit_influxdata(out_msg)
        else:
            """
            unknown strategy
            """
            rospy.logdebug("Unknown strategy %s for %s" % (self.strategy, self))
            pass

    def _submit_influxdata(self, out_msg):
        """
        Accept out_msg of type Event, calculates influx submission string,
        populates out_msg `influx` atrribute with it and submits its `influx` attribute
        (which is a string) to influxdb. Publishes Event type message about.
        """

        influx_data = self.influxdb_client.get_data_for_influx(out_msg, self.measurement)
        out_msg.influx = str(influx_data)
        rospy.logdebug("Submitting to InfluxDB: '%s'" % influx_data)
        rospy.logdebug("Publishing out_msg: %s" % out_msg)
        rospy.logdebug("Types: %s, %s, %s, %s, %s" % (type(out_msg.measurement),
                                                      type(out_msg.src_topic),
                                                      type(out_msg.type),
                                                      type(out_msg.metadata),
                                                      type(out_msg.value)))
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
                influx_data = self._submit_influxdata(out_msg)
                """
                Do resubmission only for events
                """
                with self._lock:
                    self.last_in_msg = msg
                    self.time_of_last_in_msg = time.time()
                    self.last_influx_data = influx_data
                    self.last_out_msg = out_msg
                    self.time_of_last_resubmission = None

        except EmptyIncomingMessage as ex:
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
    event_measurement_name = rospy.get_param("~event_measurement_name", 'lg_stats_event')
    metric_measurement_name = rospy.get_param("~metric_measurement_name", 'lg_stats_metric')
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
        rospy.on_shutdown(p.on_shutdown)

    # wake all processors that have strategy of average and count and make sure their buffers are emptied
    rospy.loginfo("Initializing lg_stats with: %s" % processors)

    rospy.loginfo("%s spinning ..." % ROS_NODE_NAME)
    rospy.spin()
