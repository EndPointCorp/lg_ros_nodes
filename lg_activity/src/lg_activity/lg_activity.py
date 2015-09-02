#!/usr/bin/env python

import rospy
import sys
import threading


from lg_common.helpers import unpack_activity_sources
from lg_common.helpers import list_of_dicts_is_homogenous


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
                       topic=None,
                       message_type=None,
                       strategy=None,
                       slot=None,
                       value=None,
                       value_min=None,
                       value_max=None,
                       callback=None):
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
                rospy.loginfo("Registering activity source with value=%s, min=%s, max=%s and msg attribute=%s" %\
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

    def _initialize_subscriber(self):
        """
        Performes a dirty python stuff to subscribe to a topic using
        message type provided with a string in '<module>/<msg>' format
        """
        try:
            module = self.message_type.split('/')[0]
            # e.g. interactivespaces_msgs
            message = self.message_type.split('/')[1]
            # e.g. GenericMessage
            module_obj = __import__('%s.msg' % module)
            globals()[module] = module_obj
            # now interactivespaces_msg is accessible from object space
            # we can call getattr() on it to retrieve the desired msg

            # let's get the module directly
            message_type_final = getattr(getattr(sys.modules[module], 'msg'), message)()
            rospy.Subscriber(self.topic, message_type_final, self.callback)
        except Exception, e:
            msg = "Could not load module:%s, message:%s because: %s" % (module, message, e)
            rospy.logerr(msg)
            raise ActivitySourceException

        rospy.Subscribe(topic, message_type_final, self._aggregate_message)

    def _aggregate_message(self, message):
        """
        Check for memory limits, deserialize message to python dict, append message.
        """
        if sys.getsizeof(self.messages) >= self.memory_limit:
            rospy.logwarn("%s activity source memory limit reached (%s) - discarding 2 oldest messages" % (self.topic, self.memory_limit))
            del self.messages[-1]
            del self.messages[-1]

        self._deserialize_and_append(message)

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
            slots = message.__slots__
            for attr_name in slots:
                deserialized_msg = self._get_slot_value_from_message(message)

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
        """
        if self.strategy == 'delta':
            if list_of_dicts_is_homogenous(self.messages):
                self_messages = []
                self.callback(self.topic, active=False)
                return False #if list if homogenous than there was no activity
            else:
                self.messages = []
                self.callback(self.topic, active=True)
                return True #if list is not homogenous than there was activity

        elif self.strategy == 'value':
            if self._messages_met_value_constraints():
                self.messages = []
                self.callback(self.topic, active=False)
                return False #messages met the constraints
            else:
                self.callback(self.topic, active=True)
                return True #messages didnt meet the constraints

        elif self.strategy == 'activity':
            if len(self.messages) > 0:
                self.messages = []
                self.callback(self.topic, active=True)
                return True
            else:
                self.messages = []
                self.callback(self.topic, active=False)
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
        rospy.loginfo("Instantiated following activity sources: %s using provided string: %s" % (self.sources, sources_string))

    def get_sources(self):
        return self.sources

    def get_source(self, topic):
        """
        Returns single source by topic name
        """
        try:
            source = [ source for source in self.sources if source['topic'] == topic ]
            return source[0]
        except KeyError, e:
            msg = "Could not find source %s" %s
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

    def __init__(self, publisher=None, timeout=None, sources=None):
        if (not publisher) or (not timeout) or (not source):
            msg = "Activity tracker initialized without one of the params: pub=%s, timeout=%s, source=%s" % \
                    (publisher, timeout, source)
            rospy.logerr(msg)
            raise ActivityTrackerException

        self.active = True
        self.lock = threading.RLock()
        self.initialized_sources = []
        self.activity_states = {}
        self.timeout = timeout
        if not self.timeout:
            msg = "You must specify inactivity timeout"
            rospy.logerr(msg)
            raise ActivityTrackerException(msg)
        self.sources = sources
        self.publisher = publisher
        self._validate_sources()
        self._init_activity_sources()

    def tick(self, topic_name, state):
        """
        Tick should be passed to ActivitySource.
        """
        pass

    def activity_callback(self, topic_name, state):
        """
        ActivitySource uses this callback to set it's state in ActivityTracker

        ActivityTracker keeps state for all ActivitySources with timestamps.

        If all states turned False (inactive) with respect to self.timeout then message is emitted

        If all states turned True (active) then proper message is emitted
        """
        try:
            with self.lock:
                try:
                    if self.activity_states[topic_name]['state'] == state:
                        # don't set the state if already in that state
                        pass
                    else:
                        self.activity_states[topic_name] = {"state": state, "time": rospy.get_time() }
                        rospy.loginfo("Topic name: %s state changed to %s" % (topic_name, state))
                except KeyError:
                    self.activity_states[topic_name] = {"state": state, "time": rospy.get_time() }

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
        if ((now - source['time']) >= self.timeout) and source['state'] == False:
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
        with self.lock:
            for state in self.states.values():
                if state['state'] == True and self.active == False:
                    self.publisher.publish(Bool(data=True))
                    self.active = True
                    # state turns from False (inactive) to True (active)
                    # because of first encounter of source in True state (active)
                    # this is basically a wakeup on any activity
                    return
                if self._source_has_been_inactive(state) and self.active == True:
                    # Immediately exit the loop if any of the sources
                    # was not inactive for more then self.timeout
                    return

            # After all we can announce inactivity
            self.publisher.publish(Bool(data=False))
            self.active = False
            return


    def _init_activity_sources(self):
        """
        Turns activity source dictionaries into ActivitySources objects and
        initializes them by adding their state to self.activity_states and
        providing a self.activity_callback for them

        """
        for source in self.sources:
            act = ActivitySource(topic=source['topic'],
                           message_type=source['msg_type'],
                           strategy=source['strategy'],
                           slot=source['slot'],
                           value_min=source['value_min'],
                           value_max=source['value_max'],
                           callback=self.activity_callback)
            self.initialized_sources.append(act)
        return True

    def _validate_sources(self):
        for source in self.sources:
            if type(source) != {}:
                msg = "sources argument must be a list containing ActivitySource definition dictionaries"
                rospy.logerr(msg)
                raise ActivitySourceException

    def _get_state(self, header=None):
        """
        Should return boolean activity state based on timeout and sources states
        """
        active_sources = [state for state in self.states.iteritems() if source[1] == 'active']

    def _sleep_between_checks(self):
        rospy.logdebug("Sleeping for %s seconds before writing stats" % self.timeout)
        rospy.sleep(self.timeout)
        pass
