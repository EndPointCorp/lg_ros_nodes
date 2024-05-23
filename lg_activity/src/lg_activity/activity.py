#!/usr/bin/env python3

import rospy
import sys
import threading
import json

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import list_of_dicts_is_homogenous
from lg_common.helpers import rewrite_message_to_dict
from lg_common.helpers import get_message_type_from_string
from lg_common.helpers import get_nested_slot_value
from std_msgs.msg import Bool
from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage

from lg_common.logger import get_logger
logger = get_logger('activity_class')


class ActivitySourceNotFound(Exception):
    pass


class ActivitySourceException(Exception):
    pass


class ActivityTrackerException(Exception):
    pass


class ActivitySource:
    """
    Should be instantiated with topic, message_type and strategy

    Provides:
    - subscription to topic of activity (e.g touchscreen or spacenav)
    - storage for all messages flowing on a given topic with some memory limit
    - checking for a slot if strategy is based on single attribute lookup. The slot can be nested like: angular.x
      in geometry_msgs/Twist case this will check for Twist().angular 'x' attribute
    - minimum/maximum value for 'value' strategy - everything outside of the constraints **is** activity
      for `value` kind of strategy, a slot is needed
    - memory limit for messages storage - by default 1024000 (1MB)
        e.g. 10k geometry_msg/Twist messages = 87632 bytes
    - "is_active" method that should
     - return True/False if aggregated list of events (from the specified topic)
     triggered the activity by applying the provided strategy type
     - erase aggregated messages upon "is_active" call
    """
    DELTA_MSG_COUNT = 15

    def __init__(self, memory_limit=1024000,
                 topic=None, message_type=None,
                 strategy=None, slot=None,
                 value_min=None, debug=None,
                 value_max=None, callback=None, tracker=None):
        self._check_init_args(topic, message_type, strategy, callback,
                              value_min, value_max, slot)
        self.message_type = message_type
        self.callback = callback
        self.slot = slot
        self.strategy = strategy
        self.value_min = value_min
        self.value_max = value_max
        self.debug = debug
        self.topic = topic
        self.memory_limit = memory_limit
        self.messages = []
        self.delta_msg_count = self.__class__.DELTA_MSG_COUNT
        self.tracker = tracker
        self._initialize_subscriber()
        logger.info("Initialized ActivitySource: %s" % self)

    def _check_init_args(self, topic, message_type, strategy, callback, value_min, value_max, slot):
        """
        ActivitySource is v fragile when it comes to initialization and subscriptions
        This method provides all sanity checks for that
        """
        if (not topic) or (not message_type) or (not strategy) or (not callback):
            msg = "Could not initialize ActivitySource: topic=%s, message_type=%s, strategy=%s, callback=%s" % \
                (topic, message_type, strategy, callback)
            logger.error(msg)
            raise ActivitySourceException(msg)

        if (type(topic) != str) or (type(message_type) != str):
            msg = "Topic and message type should be strings"
            logger.error(msg)
            raise ActivitySourceException(msg)

        if strategy == 'value':
            """
            For 'value' strategy we need to provide a lot of data
            """
            if value_min and value_max and slot:
                logger.info("Registering activity source with min=%s, max=%s and msg attribute=%s" %
                              (value_min, value_max, slot))
            else:
                msg = "Could not initialize 'value' stragegy for ActivitySource. All attrs are needed (min=%s, max=%s and msg attribute=%s)" % \
                    (value_min, value_max, slot)
                logger.error(msg)
                raise ActivitySourceException(msg)

    def __str__(self):
        """
        String representation of the ActivitySource - needed by pretty logs
        """
        string_representation = "<ActivitySource: slot: %s, strategy: %s, value_min:%s, value_max:%s on topic %s>" % \
            (self.slot, self.strategy, self.value_min, self.value_max, self.topic)
        return string_representation

    def __repr__(self):
        """
        __repr__ representation of the ActivitySource - needed by pretty logs
        """
        string_representation = "<ActivitySource: slot: %s, strategy: %s, value_min:%s, value_max:%s on topic %s>" % \
            (self.slot, self.strategy, self.value_min, self.value_max, self.topic)
        return string_representation

    def _initialize_subscriber(self):
        """
        Performes a dirty python stuff to subscribe to a topic using
        message type provided with a string in '<module>/<msg>' format
        """
        try:
            message_type_final = get_message_type_from_string(self.message_type)
        except Exception as e:
            msg = "Could not import module because: %s" % (e)
            logger.error(msg)
            raise ActivitySourceException

        logger.info("ActivitySource is going to subscribe topic: %s with message_type: %s" % (self.topic, message_type_final))
        self.subscriber = rospy.Subscriber(self.topic, message_type_final, self._aggregate_message)

    def _aggregate_message(self, message):
        """
        Check for memory limits, deserialize message to python dict, append message to
        ActivitySource buffer for later calculations
        """
        while sys.getsizeof(self.messages) >= self.memory_limit:
            logger.warn("%s activity source memory limit reached (%s) - discarding oldest message" % (self.topic, self.memory_limit))
            if len(self.messages) <= 1:
                logger.warn("Too small of a memory limit set... Ignoring")
                break
            del self.messages[0]

        self._deserialize_and_append(message)
        self.is_active()

    def _get_slot_value_from_message(self, message):
        """
        For every strategy, if 'slot' is specified, it can be nested e.g.:
            linear:
                x: 0.0
                y: 0.0
                z: 0.0
            angular:
                x: 0.0
                y: 0.0
                z: 0.0

        User can specify slot like 'linear.x' and this method is responsible for retrieving it
        and returning as a dictionary e.g. {'linear.x': 'some_value'}
        """

        return get_nested_slot_value(self.slot, message)

    def _deserialize_and_append(self, message):
        """
        Takes all message slots and turns the message into a dict.
        If single slot was provided than we're using only the provided one
        """

        deserialized_msg = {}

        if self.slot:
            deserialized_msg = self._get_slot_value_from_message(message)
        else:
            deserialized_msg = rewrite_message_to_dict(message)

        self.messages.append(deserialized_msg)

    def _messages_met_value_constraints(self):
        """
        Checks accumulated messages (typically 1 message)
        And finds out if value of the slot (e.g. 3.17)
        is within value_min:value_max range. If it's within
        the range, then method returns False meaning that
        values exceeded the range (making system active)

        Values outside of the value_min/value_max range make
        system become inactive.
        """
        for message in self.messages:
            value = float(list(message.values())[0])
            if value >= float(self.value_min) and value <= float(self.value_max):
                return False
        return True

    def is_active(self):
        """
        Apply strategy to self.messages:
            - delta - compares messages
            - value - checks for specific value
            - activity - checks for any messages flowing on a topic
            - message - last message sets the state and stays (using empty list for False, else True for now)
            - duration - messages with positive duration set active True with a now + duration timestamp

        Once state is asserted then call ActivityTracker to let him know
        what's the state of the source

        This method can be called from 'self' as well as from the outside of self
        """
        if self.strategy == 'delta':
            self._is_delta_active()
        elif self.strategy == 'value':
            self._is_value_active()
        elif self.strategy == 'activity':
            self._is_activity_active()
        elif self.strategy == 'message':
            self._is_message_active()
        elif self.strategy == 'duration':
            self._is_duration_active()
        else:
            logger.error("Unknown strategy: %s for activity on topic %s" % (self.strategy, self.topic))

    def _is_delta_active(self):
        """
        Checks whether all messages (of type dict) stored in the buffer are
        homogenous. If they are not then this activity source is active
        """
        if len(self.messages) < self.delta_msg_count:
            logger.debug("Not enough messages (minimum of %s) for 'delta' strategy" % self.delta_msg_count)
            return

        if list_of_dicts_is_homogenous(self.messages):
            self.messages = self.messages[-self.delta_msg_count + 1:]
            self.callback(self.topic, state=False, strategy='delta')
            return False  # if list is homogenous then there was no activity
        else:
            self.messages = self.messages[-self.delta_msg_count + 1:]
            self.callback(self.topic, state=True, strategy='delta')
            return True  # if list is not homogenous than there was activity

    def _is_value_active(self):
        """
        If current message meets value constrains then the system is active
        """
        if self._messages_met_value_constraints():
            self.callback(self.topic, state=False, strategy='value')
            self.messages = []
            return False  # messages met the constraints
        else:
            self.callback(self.topic, state=True, strategy='value')
            self.messages = []
            return True  # messages didnt meet the constraints

    def _is_activity_active(self):
        """
        Here's the place where ActivitySource triggers ActivityTracker, which is its parent
        to inform it that it's active
        """
        if len(self.messages) > 0:
            self.messages = []
            self.callback(self.topic, state=True, strategy='activity')
            return True
        else:
            self.callback(self.topic, state=False, strategy='activity')
            return False

    def _is_message_active(self):
        """keep last message. Empty list returns False TODO add other cases, empty string, null, ..."""
        if self.messages:
            try:
                self.messages = [self.messages[-1]]  # skip to last message and keep it
                if self.messages[0] and isinstance(self.messages[0], dict):
                    if self.messages[0].get('overlays', False):
                        self.callback(self.topic, state=True, strategy='message', delay=1)
                        logger.debug("Setting strems state True")
                    return True
                else:
                    logger.warning("Ignoring message, expected a dict, got %"  % type(self.messages[-1]))
            except (IndexError, KeyError, ValueError, TypeError) as e:
                logger.exception("error checking stream messages: %" % e)
        self.messages = []
        self.callback(self.topic, state=False, strategy='message')
        logger.debug("Streams are off")
        return False

    def _is_duration_active(self):
        """check scene message for duration or own state if no message"""
        if not self.tracker.active:  # ignore attract_loop playing
            self.messages = []
            return False
        if self.messages:
            try:
                if self.messages[-1] and isinstance(self.messages[-1], dict):
                    duration = self.messages[-1].get('message.duration', 0)  # skip to last message duration
                    if isinstance(duration, int) or (isinstance(duration, str) and duration.isdigit()):
                        duration = int(duration)
                        if duration > 0 and duration != 666:
                            self.messages = []
                            self.callback(self.topic, state=True, strategy='duration', delay=duration)
                            logger.info("Setting scene_duration state True delayed by %s seconds duration " % duration)
                            return True
                    else:
                        logger.warning("Ignoring duration, it should be a number of seconds, message is: ", self.messages[-1])
                else:
                    logger.warning("Ignoring message, expected a dict, got %"  % type(self.messages[-1]))
            except (IndexError, KeyError, ValueError, TypeError) as e:
                logger.exception("error getting duration from message: %" % e)

            self.messages = []
            self.callback(self.topic, state=False, strategy='duration')
            return False

        try:  # no messages, get self state
            if self.tracker.activity_states[self.topic]["time"] > rospy.get_time():  # if own time is in the future
                logger.debug("scene_duration returns True, %s  >  %s" % (self.tracker.activity_states[self.topic]["time"], rospy.get_time()))
                return True
            else:
                logger.debug("scene_duration returns and calls False, %s ~<~  %s" % (self.tracker.activity_states[self.topic]["time"], rospy.get_time()))
        except Exception as e:  #no previous timestamp so inactive
            logger.debug("init scene_duration, return and call False")
        self.callback(self.topic, state=False, strategy='duration')
        return False

class ActivitySourceDetector:
    """
    Provides a getter for a dictionary of topics, messages and strategies e.g.:
    example source:

    source = { "topic": "/touchscreen/touch",
               "message_type": "interactivespaces_msgs/String",
               "strategy": "activity",
               "slot": None,
               "value_min": None,
               "value_max": None
             }
    """
    def __init__(self, sources_string):
        self.sources = unpack_activity_sources(sources_string)
        logger.info("Initialized ActivitySourceDetector: %s" % self)

    def __str__(self):
        string_representation = "<ActivitySourceDetector: sources: %s" % self.sources
        return string_representation

    def __repr__(self):
        string_representation = "<ActivitySourceDetector: sources: %s" % self.sources
        return string_representation

    def get_sources(self):
        return self.sources

    def get_source(self, topic):
        """
        Returns single source by topic name
        """
        try:
            source = [source for source in self.sources if source['topic'] == topic]
            return source[0]
        except KeyError as e:
            msg = "Could not find source %s because %s" % (source, e)
            logger.error(msg)
            raise ActivitySourceNotFound(msg)


class ActivityTracker:
    """
    Provides callback that should be passed to ActivitySources.

    Each ActivitySource will call the callback upon the activity
    coming on a topic that the ActivitySource is watching.

    Provides a topic /activity/active and emits a active: True|False message
    depending on the activity state change

    Keeps the state of activity. State should be 'active: true' by default

    Provides service for checking the activity state

    """

    def __init__(self, publisher=None, stats_activity_publisher=None, timeout=10, stats_activity_timeout=120, sources=None, debug=None):
        if (not publisher) or (not stats_activity_publisher) or (not timeout) or (not sources) or (not stats_activity_timeout):
            msg = "Activity tracker initialized without one of the params: pub=%s, stats_activity_publisher=%s, timeout=%s, stats_activity_timeout=%s, sources=%s" % \
                (publisher, stats_activity_publisher, timeout, stats_activity_timeout, sources)
            logger.error(msg)
            raise ActivityTrackerException

        self.active = True
        self.stats_active = True
        self._lock = threading.Lock()
        self.initialized_sources = []
        self.activity_states = {}
        self.debug = debug
        self.timeout = timeout
        self.stats_activity_timeout = stats_activity_timeout
        if not self.timeout:
            msg = "You must specify inactivity timeout"
            logger.error(msg)
            raise ActivityTrackerException(msg)
        self.sources = sources
        self.publisher = publisher
        self.publisher.publish(Bool(data=True))  # init the state with True (active)
        self.stats_activity_publisher = stats_activity_publisher
        rospy.sleep(0.1)
        self.stats_activity_publisher.publish(Bool(data=True))  # init the state with True (active)
        self._validate_sources()
        self._init_activity_sources()
        logger.info("Initialized ActivityTracker: %s" % self)

    def __str__(self):
        string_representation = "<ActivityTracker: sources: %s, initialized_sources: %s, timeout: %s, stats_activity_timeout: %s, publisher: %s, stats_activity_publisher: %s" % \
            (self.sources, self.initialized_sources, self.timeout, self.stats_activity_timeout, self.publisher, self.stats_activity_publisher)
        return string_representation

    def __repr__(self):
        string_representation = "<ActivityTracker: sources: %s, initialized_sources: %s, timeout: %s, stats_activity_timeout: %s, publisher: %s, stats_activity_publisher: %s" % \
            (self.sources, self.initialized_sources, self.timeout, self.stats_activity_timeout, self.publisher, self.stats_activity_publisher)
        return string_representation

    def _get_activity_status(self, _):
        """
        ROS service method for retrieving activity state

        """
        with self._lock:
            return self.active

    def activity_callback(self, topic_name=None, state=True, strategy=None, delay=0):
        """
        ActivitySource uses this callback to set it's state in ActivityTracker

        ActivityTracker keeps state for all ActivitySources with timestamps.

        If all states turned False (inactive) with respect to self.timeout then message is emitted

        If all states turned True (active) then proper message is emitted
        """
        with self._lock:
            try:
                if topic_name not in self.activity_states:
                    self.activity_states[topic_name] = {"state": state, "time": rospy.get_time() + delay}
                    logger.info("Initialize Topic name: %s with state: %s and delay: %s" % (topic_name, state, delay))
                    return True

                try:
                    if self.activity_states[topic_name]['state'] == state:
                        if delay > 0:  # use delay as a force update param
                            self.activity_states[topic_name] = {"state": state, "time": rospy.get_time() + delay}
                            logger.debug("Updated Topic name: %s in state: %s with delay: %s" % (topic_name, state, delay))

                        logger.debug("State of %s didn't change" % topic_name)
                        return True

                    logger.debug("Updating: %s state to %s" % (topic_name, state))
                except KeyError:
                    logger.error("Initializing: %s with state: %s" % (topic_name, state))

                self.activity_states[topic_name] = {"state": state, "time": rospy.get_time() + delay}
                self._check_states()
                return True

            except Exception as e:
                logger.debug("Activity callback with state %s for topic name %s failed!: %s" % (state, topic_name, e))

            return False

    def _source_is_active(self, source, timeout=120):
        """
        Checks whether source was active for last `self.timeout` seconds
        which basically means that we can consider this source to be active

        Accepts source state dict eg.:
        {"state": True, "time": <unix timestamp> }

        All following constraints need to be met to consider source active:
        a) source is in True (active) state

        All following constraints need to be to consider source inactive:
        b) source is in False (inactive) state
        c) last change of source's state was more than `self.timeout` seconds ago
        """

        now = rospy.get_time()

        if source['state'] is True:
            logger.debug("ActivitySource %s is active" % source)
            return True

        if source['state'] is False and ((now - source['time']) >= timeout):
            logger.debug("ActivitySource %s is inactive" % source)
            return False
        else:
            logger.debug("ActivitySource %s is in inactivity timeout" % source)
            return True

    def _check_states(self):
        """
        Emits a proper activity message if all states were in False or True for
        certain period of time
        'state' == True means "active"

        1. check for activity when in inactive state (easiest and fastest)
        2. carefully check for all states
        """

        now = rospy.get_time()
        self.sources_active_within_timeout = {state_name: state for state_name, state in self.activity_states.items() if self._source_is_active(state, timeout=self.timeout)}

        if self.sources_active_within_timeout and (not self.active):
            self.active = True
            self.publisher.publish(Bool(data=True))
            logger.info("State turned from False to True because of state: %s" % self.sources_active_within_timeout)
            logger.debug("States: %s" % self.activity_states)
        elif (not self.sources_active_within_timeout) and self.active:
            self.active = False
            self.publisher.publish(Bool(data=False))
            logger.info("State turned from True to False because no sources were active within the timeout")
            logger.debug("States: %s" % self.activity_states)
        else:
            logger.debug("Activity state unchanged. Active sources: %s, state: %s, activity_states: %s" % (self.sources_active_within_timeout, self.active, self.activity_states))


        self.sources_active_within_stats_timeout = {state_name: state for state_name, state in self.activity_states.items() if self._source_is_active(state, timeout=self.stats_activity_timeout)}

        if self.sources_active_within_stats_timeout and (not self.stats_active):
            self.stats_active = True
            self.stats_activity_publisher.publish(Bool(data=True))
            logger.info("Stats activity state turned from False to True because of state: %s" % self.sources_active_within_stats_timeout)
            logger.info("States: %s" % self.activity_states)
        elif (not self.sources_active_within_stats_timeout) and self.stats_active:
            self.stats_active = False
            self.stats_activity_publisher.publish(Bool(data=False))
            logger.info("Stats acitivity state turned from True to False because no sources were active within the timeout")
            logger.info("States: %s" % self.activity_states)
        else:
            logger.debug("Message criteria not met. Active sources: %s, state: %s, activity_states: %s" % (self.sources_active_within_stats_timeout, self.stats_active, self.activity_states))



    def _init_activity_sources(self):
        """
        Turns activity source dictionaries into ActivitySources objects and
        initializes them by adding their state to self.activity_states and
        providing a self.activity_callback for them

        """
        for source in self.sources:
            act = ActivitySource(
                topic=source['topic'], message_type=source['message_type'],
                strategy=source['strategy'], slot=source['slot'],
                value_min=source['value_min'], value_max=source['value_max'],
                callback=self.activity_callback, debug=self.debug, tracker=self)
            self.initialized_sources.append(act)
        return True

    def _validate_sources(self):
        for source in self.sources:
            if type(source) != dict or type(self.sources) != list:
                msg = "sources argument must be a list containing ActivitySource definition dictionaries but was: %s" % self.sources
                logger.error(msg)
                raise ActivitySourceException

    def _get_state(self, header=None):
        """
        Should return boolean activity state based on timeout and sources states
        Used mainly for Service for debugging purposes
        """
        return json.dumps(self.activity_states)

    def poll_activities(self):
        for source in self.initialized_sources:
            source.is_active()
        self._check_states()

    def _sleep_between_checks(self):
        # rospy.sleep(self.timeout)
        pass
