#!/usr/bin/env python

import rospy
import sys

from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import list_of_dicts_is_homogenous
from lg_common.helpers import rewrite_message_to_dict
from lg_common.helpers import get_message_type_from_string
from lg_activity.msg import ActivityState
from lg_activity.srv import ActivityStates
from std_msgs.msg import Bool


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
    def __init__(self, memory_limit=1024000,
                 topic=None, message_type=None,
                 strategy=None, slot=None,
                 value=None, value_min=None,
                 value_max=None, callback=None):
        if (not topic) or (not message_type) or (not strategy) or (not callback):
            msg = "Could not initialize ActivitySource: topic=%s, message_type=%s, strategy=%s, callback=%s" % \
                (topic, message_type, strategy, callback)

            rospy.logerr(msg)
            raise ActivitySourceException(msg)

        if (type(topic) != str) or (type(message_type) != str):
            msg = "Topic and message type should be strings"

            rospy.logerr(msg)
            raise ActivitySourceException(msg)

        if (value_min and value_max) and (not slot):
            msg = "You must provide slot when providing value_min and value_max"
            rospy.logerr(msg)
            raise ActivitySourceException(msg)

        self.message_type = message_type
        self.callback = callback
        self.slot = slot
        self.value = value
        self.strategy = strategy
        self.value_min = value_min
        self.value_max = value_max

        if self.strategy == 'value':
            """
            For 'value' strategy we need to provide a lot of data
            """
            if self.value and self.value_min and self.value_max and self.slot:
                rospy.loginfo("Registering activity source with value=%s, min=%s, max=%s and msg attribute=%s" %
                              (self.value, self.value_min, self.value_max, self.slot))
            else:
                msg = "Could not initialize 'value' stragegy for ActivitySource. All attrs are needed (value=%s, min=%s, max=%s and msg attribute=%s)" % \
                    (self.value, self.value_min, self.value_max, self.slot)
                rospy.logerr(msg)
                raise ActivitySourceException(msg)

        self.topic = topic
        self.memory_limit = memory_limit
        self.messages = []
        self._initialize_subscriber()
        rospy.loginfo("Initialized ActivitySource: %s" % self)

    def __str__(self):
        string_representation = "<ActivitySource: slot: %s, value:%s, strategy: %s, value_min:%s, value_max:%s on topic %s>" % \
            (self.slot, self.value, self.strategy, self.value_min, self.value_max, self.topic)
        return string_representation

    def __repr__(self):
        string_representation = "<ActivitySource: slot: %s, value:%s, strategy: %s, value_min:%s, value_max:%s on topic %s>" % \
            (self.slot, self.value, self.strategy, self.value_min, self.value_max, self.topic)
        return string_representation

    def _initialize_subscriber(self):
        """
        Performes a dirty python stuff to subscribe to a topic using
        message type provided with a string in '<module>/<msg>' format
        """
        try:
            message_type_final = get_message_type_from_string(self.message_type)
        except Exception, e:
            msg = "Could not import module because: %s" % (e)
            rospy.logerr(msg)
            raise ActivitySourceException

        rospy.loginfo("ActivitySource is going to subscribe topic: %s with message_type: %s" % (self.topic, message_type_final))
        self.subscriber = rospy.Subscriber(self.topic, message_type_final, self._aggregate_message)

    def _aggregate_message(self, message):
        """
        Check for memory limits, deserialize message to python dict, append message.
        """
        if sys.getsizeof(self.messages) >= self.memory_limit:
            rospy.logwarn("%s activity source memory limit reached (%s) - discarding 2 oldest messages" % (self.topic, self.memory_limit))
            del self.messages[-1]
            del self.messages[-1]

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
        and returning as a dictionary e.g. {'linear.x'
        """

        slot_tree = self.slot.split('.')

        if len(slot_tree) == 1:
            return {self.slot: getattr(message, self.slot)}

        elif len(slot_tree) > 1:
            for slot_number in xrange(0, len(slot_tree)):
                if slot_number == 0:
                    deserialized_msg = getattr(message, slot_tree[slot_number])
                else:
                    deserialized_msg = getattr(deserialized_msg, slot_tree[slot_number])
        else:
            msg = "Wrong slot_tree provided: %s" % self.slot
            rospy.logerr(msg)
            raise ActivitySourceException(msg)

        return {self.slot: deserialized_msg}

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

    def _messages_met_value_constraints():
        for message in self.messages:
            if not (message.values()[0] >= value_min) or not (message.values()[0]) <= value_max:
                return False
        return True

    def is_active(self):
        """
        Apply strategy to self.messages:
            - delta - compares messages
            - value - checks for specific value
            - activity - checks for any messages flowing on a topic

        Once state is asserted then call ActivityTracker to let him know
        what's the state of the source

        This method can be called from 'self' as well as from the outside of self
        """
        if self.strategy == 'delta':
            if len(self.messages) >= 5:
                if list_of_dicts_is_homogenous(self.messages):
                    self.messages = []
                    self.callback(self.topic, state=False, strategy='delta')
                    return False  # if list if homogenous than there was no activity
                else:
                    self.messages = []
                    self.callback(self.topic, state=True, strategy='delta')
                    return True  # if list is not homogenous than there was activity
            else:
                rospy.logdebug("Not enough messages (minimum of 5) for 'delta' strategy")

        elif self.strategy == 'value':
            if len(self.messages) >= 1:
                if self._messages_met_value_constraints():
                    self.messages = []
                    self.callback(self.topic, state=False, strategy='value')
                    return False  # messages met the constraints
                else:
                    self.callback(self.topic, state=True, strategy='value')
                    return True  # messages didnt meet the constraints
            else:
                rospy.loginfo("Not enough messages (minimum of 1) for 'value' strategy")

        elif self.strategy == 'activity':
            if len(self.messages) > 0:
                self.messages = []
                self.callback(self.topic, state=True, strategy='activity')
                return True
            else:
                self.messages = []
                self.callback(self.topic, state=False, strategy='activity')
                return False
        else:
            rospy.logerr("Unknown strategy: %s for activity on topic %s" % (self.strategy, self.topic))


class ActivitySourceDetector:
    """
    Provides a getter for a dictionary of topics, messages and strategies e.g.:
    example source:

    source = { "topic": "/touchscreen/touch",
               "msg_type": "interactivespaces_msgs/String",
               "strategy": "activity",
               "slot": None,
               "value_min": None,
               "value_max": None
             }
    """
    def __init__(self, sources_string):
        self.sources = unpack_activity_sources(sources_string)
        rospy.loginfo("Initialized ActivitySourceDetector: %s" % self)

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
        except KeyError, e:
            msg = "Could not find source %s" % s
            rospy.logerr(msg)
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

    def __init__(self, publisher=None, timeout=10, sources=None):
        if (not publisher) or (not timeout) or (not sources):
            msg = "Activity tracker initialized without one of the params: pub=%s, timeout=%s, sources=%s" % \
                (publisher, timeout, sources)
            rospy.logerr(msg)
            raise ActivityTrackerException

        self.active = True
        self.initialized_sources = []
        self.activity_states = {}
        self.timeout = timeout
        if not self.timeout:
            msg = "You must specify inactivity timeout"
            rospy.logerr(msg)
            raise ActivityTrackerException(msg)
        self.sources = sources
        self.publisher = publisher
        self.publisher.publish(Bool(data=True)) #init the state with True (active)
        self._validate_sources()
        self._init_activity_sources()
        rospy.loginfo("Initialized ActivityTracker: %s" % self)

    def __str__(self):
        string_representation = "<ActivityTracker: sources: %s, initialized_sources: %s, timeout: %s, publisher: %s" % \
            (self.sources, self.initialized_sources, self.timeout, self.publisher)
        return string_representation

    def __repr__(self):
        string_representation = "<ActivityTracker: sources: %s, initialized_sources: %s, timeout: %s, publisher: %s" % \
            (self.sources, self.initialized_sources, self.timeout, self.publisher)
        return string_representation

    def tick(self, topic_name, state):
        """
        Tick should be passed to ActivitySource.
        """
        pass


    def activity_callback(self, topic_name=None, state=True, strategy=None):
        """
        ActivitySource uses this callback to set it's state in ActivityTracker

        ActivityTracker keeps state for all ActivitySources with timestamps.

        If all states turned False (inactive) with respect to self.timeout then message is emitted

        If all states turned True (active) then proper message is emitted
        """
        try:
            try:
                if self.activity_states[topic_name]['state'] == state and strategy != 'activity':
                    rospy.logdebug("State of %s didnt change" % topic_name)
                else:
                    self.activity_states[topic_name] = {"state": state, "time": rospy.get_time()}
                    rospy.loginfo("Topic name: %s state changed to %s" % (topic_name, state))
            except KeyError:
                rospy.loginfo("Initializing topic name state: %s" % topic_name)
                self.activity_states[topic_name] = {"state": state, "time": rospy.get_time()}

            self._check_states()
            return True
        except Exception, e:
            rospy.logerr("activity_callback for %s failed because %s" % (topic_name, e))
            return False

    def _source_has_been_inactive(self, source):
        """
        Checks whether source was inactive for more seconds than self.timeout
        Accepts source state dict eg.:
        {"state": True, "time": <unix timestamp> }
        """
        now = rospy.get_time()
        if ((now - source['time']) >= self.timeout):
            return True
        else:
            return False

    def _source_become_active(self, source):
        """
        Checks whether source become active less then self.timeout
        Accepts source state dict eg.:
        {"state": True, "time": <unix timestamp> }
        """
        now = rospy.get_time()
        if ((now - source['time']) <= self.timeout):
            return True
        else:
            return False

    def _check_states(self):
        """
        Emits a proper activity message if all states were in False or True for
        certain period of time
        'state' == True means "active"

        1. check for activity when in inactive state (easiest and fastest)
        2. carefully check for all states


        """
        now = rospy.get_time()
        self.sources_active_within_timeout = {state_name:state for state_name, state in self.activity_states.iteritems() if (now - self.timeout) < state['time']}

        if self.sources_active_within_timeout and (not self.active):
            self.active = True
            self.publisher.publish(Bool(data=True))
            rospy.loginfo("State turned from False to True because of state: %s" % self.sources_active_within_timeout)
            rospy.loginfo("States: %s" % self.activity_states)
        elif (not self.sources_active_within_timeout) and self.active:
            self.active = False
            self.publisher.publish(Bool(data=False))
            rospy.loginfo("State turned from True to False because of state: %s" % self.sources_active_within_timeout)
            rospy.loginfo("States: %s" % self.activity_states)
        else:
            rospy.logdebug("Message criteria not met. Active sources: %s, state: %s, activity_states: %s" % (self.sources_active_within_timeout, self.active, self.activity_states))


    def _init_activity_sources(self):
        """
        Turns activity source dictionaries into ActivitySources objects and
        initializes them by adding their state to self.activity_states and
        providing a self.activity_callback for them

        """
        for source in self.sources:
            act = ActivitySource(
                topic=source['topic'], message_type=source['msg_type'],
                strategy=source['strategy'], slot=source['slot'],
                value_min=source['value_min'], value_max=source['value_max'],
                callback=self.activity_callback)
            self.initialized_sources.append(act)
        return True

    def _validate_sources(self):
        for source in self.sources:
            if type(source) != dict or type(self.sources) != list:
                msg = "sources argument must be a list containing ActivitySource definition dictionaries but was: %s" % self.sources
                rospy.logerr(msg)
                raise ActivitySourceException

    def _get_state(self, header=None):
        """
        Should return boolean activity state based on timeout and sources states
        Used mainly for Service for debugging purposes
        """
        activity_states_list = []

        for state_name, state_data in self.activity_states.iteritems():
            a = ActivityState(topic=state_name,
                              state=state_data['state'],
                              timestamp=state_data['time'])

            activity_states_list.append(a)

        return activity_states_list

    def _sleep_between_checks(self):
        #rospy.sleep(self.timeout)
        pass
