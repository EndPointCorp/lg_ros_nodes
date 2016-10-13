import os
import sys
import json
import rospy
import base64
import urllib
import hashlib
import urlparse
import random
import string

from interactivespaces_msgs.msg import GenericMessage


class PublisherSubscriberConnectionsException(Exception):
    pass


class WrongActivityDefinition(Exception):
    pass


class DependencyException(Exception):
    pass


class SlotUnpackingException(Exception):
    pass


def escape_asset_url(asset_url):
    """
    Replace all non-alnum characters with underscore
    """
    ss = ''
    try:
        for ch in asset_url:
            if str.isalnum(str(ch)):
                ss += ch
            else:
                ss += "_"
        return ss
    except TypeError:
        rospy.logerr("Got invalid type when trying to escape assets url %s" % asset_url)
        return ss


def add_url_params(url, **params):
    """
    Add GET params to the url
    url: string of the URL
    params: dict containing the keyword arguments

    return string, updated url with the params
    """
    url_parts = list(urlparse.urlparse(url))
    query_dict = dict(urlparse.parse_qsl(url_parts[4]))
    query_dict.update(params)

    # 4th index is the query params position
    url_parts[4] = urllib.urlencode(query_dict)

    return urlparse.urlunparse(url_parts)


def geometry_compare(adhoc_browser_message, managed_adhoc_browser_instance):
    """
    Accepts adhoc browser message and ManagedAdhocBrowser instnace
    and compares geometry of these
    returns bool
    """
    geometry_match = (
        int(adhoc_browser_message.geometry.x) == int(managed_adhoc_browser_instance.geometry.x) and
        int(adhoc_browser_message.geometry.y) == int(managed_adhoc_browser_instance.geometry.y) and
        int(adhoc_browser_message.geometry.width) == int(managed_adhoc_browser_instance.geometry.width) and
        int(adhoc_browser_message.geometry.height) == int(managed_adhoc_browser_instance.geometry.height))

    return geometry_match


def url_compare(a0, b0):
    """
    Compare two urls for equivalence. Order of query parameters is ignored.

    Args:
        a0 (str)
        b0 (str)

    Returns:
        True if the urls are equivalent.
    """
    a = urlparse.urlparse(a0)
    b = urlparse.urlparse(b0)

    if a.scheme != b.scheme:
        return False

    if a.netloc != b.netloc:
        return False

    if a.path != b.path:
        return False

    if a.params != b.params:
        return False

    if a.fragment != b.fragment:
        return False

    qa = urlparse.parse_qs(a.query)
    qb = urlparse.parse_qs(b.query)
    if qa != qb:
        return False

    return True


def geometry_compare(adhoc_browser_message, managed_adhoc_browser_instance):
    """
    Accepts adhoc browser message and ManagedAdhocBrowser instnace
    and compares geometry of these
    returns bool
    """
    geometry_match = (
        int(adhoc_browser_message.geometry.x) == int(managed_adhoc_browser_instance.geometry.x) and
        int(adhoc_browser_message.geometry.y) == int(managed_adhoc_browser_instance.geometry.y) and
        int(adhoc_browser_message.geometry.width) == int(managed_adhoc_browser_instance.geometry.width) and
        int(adhoc_browser_message.geometry.height) == int(managed_adhoc_browser_instance.geometry.height))

    return geometry_match


def write_log_to_file(message):
    """
    Write a log line to a file - don't use it in production!
    """
    with open('/tmp/rostests.out', 'a') as rostest_file:
        rostest_file.write(message)
        rostest_file.write('\n')


def generate_cookie(assets):
    """ Accept a list of URLs, turn them to nice slugs and return a string e.g.
            'blah.kml,zomg__kml,whatever.kml'
        rtype: str
    """
    cookie = (',').join([escape_asset_url(asset) for asset in assets])
    rospy.logdebug("Generated cookie = %s after new state was set" % cookie)
    return cookie


def get_app_instances_ids(instances):
    """
    Accepts a dictionary of id: AppInstance and returns a set of keys
    """
    return set(instances.keys())


def get_app_instances_to_manage(current_instances, incoming_instances, manage_action=None):
    """
    Accepts two sets of app ids (current and new) and returns a list of instances
    that should be created, removed or updated on the basis of `manage_action` parameter
    """

    if manage_action == 'remove':
        instances_ids_to_remove = current_instances - incoming_instances
        return list(instances_ids_to_remove)
    elif manage_action == 'update':
        instances_ids_to_update = current_instances & incoming_instances
        return list(instances_ids_to_update)
    elif manage_action == 'create':
        instances_ids_to_create = incoming_instances - current_instances
        return list(instances_ids_to_create)
    else:
        rospy.logerr("No action provided for get_app_instances_to_manage")
        return False


def load_director_message(message):
    """
    json.loads the director message, or warns and throws an exception
    """
    ret = {}
    try:
        ret = json.loads(message.message)
    except (ValueError, SyntaxError) as e:
        rospy.logwarn("Could not parse json message in helpers.load_director_message")
        rospy.logwarn("Message: %s" % message)
        raise e

    return ret


def extract_first_asset_from_director_message(message, activity_type, viewport):
    """
    Extracts **single** (first) asset and geometry and activity_config
    for given activity and viewport, e.g. all assets for browser or all KMLs for GE.
    rtype: list

    Example director message:
    message: {
              "description":"",
              "duration":30,
              "name":"Browser service test",
              "resource_uri":"/director_api/scene/browser-service-test/",
              "slug":"browser-service-test",
              "windows":[
                {
                "activity_config": { "some": "stuffz"},
                "activity":"browser",
                "assets":["https://www.youtube.com/watch?v=un8FAjXWOBY"],
                "height":600,
                "presentation_viewport":"center",
                "width":800,
                "x_coord":100,
                "y_coord":100
                }
                ]
            }
    """
    message = load_director_message(message)
    if not message:
        return []

    rospy.logdebug("Message: %s, activity_type: %s, viewport: %s" % (message, activity_type, viewport))
    assets = []
    for window in message.get('windows', []):
        if (window.get('activity') == activity_type) and (window.get('presentation_viewport') == viewport):
            asset_object = {}
            asset_object['path'] = str(window['assets'][0])
            asset_object['x_coord'] = window['x_coord']
            asset_object['y_coord'] = window['y_coord']
            asset_object['height'] = window['height']
            asset_object['width'] = window['width']
            asset_object['on_finish'] = 'nothing'

            activity_config = window.get('activity_config', {})

            if activity_config:
                asset_object['activity_config'] = window['activity_config']
                # TODO(wz):
                # - this needs to go away - all attribs should be
                #  kept under activity_config
                # - onFinish should be changed to on_finish on ros_cms side
                on_finish = activity_config.get('onFinish', None)
                if on_finish:
                    asset_object['on_finish'] = on_finish
                google_chrome = activity_config.get('google_chrome', None)
                if google_chrome:
                    asset_object['on_finish'] = on_finish

            assets.append(asset_object)
        else:
            rospy.logdebug("Message was not directed at activity %s on viewport %s" % (window.get('activity'), window.get('presentation_viewport')))

    rospy.logdebug("Returning assets: %s" % assets)
    return assets


def find_window_with_activity(scene, activity):
    """
    Returns the window who's activity == $activity, or {}
    """
    for window in scene.get('windows', []):
        if window.get('activity') == activity:
            return window

    return {}


def get_first_activity_from_scene(scene):
    windows = scene.get('windows', [])
    if windows:
        return windows[0].get('activity', '')
    return None


def get_all_activities_from_scene(scene):
    windows = scene.get('windows', [])
    activities = []
    for window in windows:
        activity = window.get('activity', None)
        if activity:
            activities.append(activity)
    return activities


def has_activity(scene, activity):
    return activity in get_all_activities_from_scene(scene)


def get_first_asset_from_activity(scene, activity):
    """
    Returns just one asset from the first matching activity, or None

    scene is a json loaded GenericMessage

    This is useful for streetview / pano activity types
    """
    window = find_window_with_activity(scene, activity)

    return window.get('assets', [None])[0]


def on_new_scene(cb):
    """
    Sets up a subscriber for director messages, unwraps and loads
    the message before calling the cb
    """
    dh = DirectorHandler(cb)
    return rospy.Subscriber('/director/scene', GenericMessage,
                            dh.handle_message)


class DirectorHandler:
    def __init__(self, callback):
        self.callback = callback

    def handle_message(self, msg):
        d = load_director_message(msg)
        self.callback(d)


def list_of_dicts_is_homogenous(dicts_list):
    """
    Returns True if dictionary values are all the same
    Returns False if more than one value is different
    """
    previous_item = dicts_list[0]
    for item in dicts_list:
        if previous_item != item:
            return False
    return True


def rewrite_message_to_dict(message):
    """
    Converts interactivespaces message (or message containing str type) to dict
    """
    deserialized_message = {}
    slots = message.__slots__
    if slots.__class__ == str:
        slots = [slots]
    for slot in slots:
        deserialized_message[slot] = getattr(message, slot)
    return deserialized_message


def message_is_nonzero(incoming_message):
    """
    returns True if all slots and subslots are nonzero
    False otherwise

    Note: tested only with geometry messages Twist()
    """
    slots_objects = incoming_message.__reduce__()[2]
    for slot_object in slots_objects:
        value_list = slot_object.__reduce__()[2]
        # print value_list
        for value in value_list:
            # print value
            if value != 0:
                return True
    return False


def unpack_activity_sources(sources_string):
    """
    Multi-purpose method, named after historically one of its first applications.
    It unpacks sources configuration strings from roslaunch config files, from
    ROS nodes definition.

    Sources string format:

    <topic_name>:<message_type>-<slot[.sub_slot.sub_sub_slot>]:<strategy>[-<value_min><value_max>|<value>]

    - topic_name: topic which ActivitySource will listen for incoming messages
    - message_type: type of message on the given topic
    - slot - for complex messages you may define one attribute of the message that
        will be considered during checks. For nested attributes, delimit slots by dots.
    - strategy - delta, value, activity, average and count. For "value" strategy you
        may define min/max values or just a single value.
    It will make sense only when slot was defined.

    Strategies info:
     - delta means that activity is generated only if messages are different. Makes
        sense with spacenav twist which emits 0,0,0 when no one's touching it
     - value is used for value tresholding - generate activity only when value of slot met min/max constraints,
        or just a single value.
     - activity - triggers activity on **every** message
     - average - not used to activity tracking - passes information about what should be done with values
     - count - same as above

    Examples:

    - source string: '/touchscreen/touch:interactivespaces_msgs/GenericMessage:activity'

    result: source = { "topic": "/touchscreen/touch",
               "message_type": "interactivespaces_msgs/GenericMessage",
               "strategy": "activity",
               "slot": None,
               "value_min": None,
               "value_max": None,
               "value": None
             }

    - source string:'/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5':

    result: source = { "topic": "/proximity_sensor/distance",
               "message_type": "sensor_msgs/Range-range",
               "strategy": "value",
               "slot": range,
               "value_min": "0",
               "value_max": "2.5",
               "value": None
             }

    NOTE: min/max values is the range within which active == True

    - source string:'/proximity_sensor/distance:sensor_msgs/Range-range:average':

    result: source = { "topic": "/proximity_sensor/distance",
               "message_type": "sensor_msgs/Range",
               "strategy": "average",
               "slot": range,
               "value_min": None,
               "value_max": None,
               "value": None
             }

    To define multiple sources use semicolon e.g.:

    '/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5
    /touchscreen/touch:interactivespaces_msgs/GenericMessage:delta'

    and a list of dictionaries will be returned.

    """
    sources = []
    bare_sources = sources_string.split(';')
    for source_string in bare_sources:
        single_source = {}
        source_fields = source_string.split(':')

        topic = source_fields[0]
        try:
            message_type = source_fields[1].split('-')[0]
        except IndexError:
            message_type = source_fields[1]

        try:
            slot = source_fields[1].split('-')[1]
        except IndexError:
            slot = None

        try:
            strategy = source_fields[2].split('-')[0]
        except IndexError:
            strategy = source_fields[2]

        try:
            values = source_fields[2].split('-')[1].split(',')
            try:
                value_min = values[0]
                value_max = values[1]
                value = None
            except IndexError:
                rospy.loginfo("Detected a singe value from source_string: %s" % source_string)
                value = values[0]
                value_min = None
                value_max = None
        except IndexError:
            rospy.loginfo("Could not get value_min/value_max nor single value from sources for source_string: %s" % source_string)
            value_min, value_max, value = None, None, None

        single_source['topic'] = topic
        single_source['message_type'] = message_type
        single_source['strategy'] = strategy
        single_source['slot'] = slot
        single_source['value_min'] = value_min
        single_source['value_max'] = value_max
        single_source['value'] = value
        sources.append(single_source)

    return sources


def build_source_string(topic, message_type, strategy, slot=None, value_min=None, value_max=None):
    """
    Builds source strings with some wizardry
    """
    # both are required
    ret = topic + ':' + message_type
    # only add sub slots if parent slots exist
    if slot:
        ret += '-' + slot
    # required
    ret += ':' + strategy
    # only do min & max if both are there
    if value_min and value_max:
        ret += '-' + str(value_min) + ',' + str(value_max)

    return ret


def check_registration(e):
    """\
    Shuts down this ROS node if it is not registered on the master.
    This will effectively kill the director each time the ROS master is
    restarted, preventing silent and subtle publishing failure.

    This should cause a shutdown *only* if the master can be contacted and the
    node is not registered.
    """
    import rosnode
    try:
        nodes = rosnode.get_node_names()
    except rosnode.ROSNodeIOException:
        rospy.logdebug("Could not contact master for registration check")
        return
    if rospy.get_name() not in nodes:
        rospy.logwarn("Node no longer registered, shutting down")
        rospy.signal_shutdown("Node no longer registered")
        os.kill(os.getpid(), signal.SIGTERM)


def begin_checking_registration(interval=1):
    """
    Periodically check the health of this node on the master.
    """
    rospy.Timer(rospy.Duration(interval), _check_registration)


def next_scene_uri(presentation, scene):
    """
    Read two JSON-encoded strings: a Presentation and a Scene.
    Decode, find the Scene within the Presentation's script,
    then return the URI for the next Scene in the script.
    """
    try:
        resource_uri = json.loads(scene)['resource_uri']
        scenes = json.loads(presentation)['scenes']
        script = map(lambda x: x['resource_uri'], scenes)
    except KeyError:
        return None

    try:
        return script[script.index(resource_uri) + 1]
    except IndexError:
        rospy.loginfo("Already at last Scene in this Presentation.")
        return None


def get_message_type_from_string(string):
    """
    Return msg_type module (e.g. GenericMessage) from string like 'interactivespaces_msgs/GenericMessage'
    """
    module = string.split('/')[0]
    # e.g. 'interactivespaces_msgs'
    message = string.split('/')[1]
    # e.g. GenericMessage
    module_obj = __import__('%s.msg' % module)
    globals()[module] = module_obj
    message_type_final = getattr(getattr(sys.modules[module], 'msg'), message)
    return message_type_final


def x_available(timeout=None):
    if not timeout:
        return
    import commands

    while timeout >= 0:
        x_check = commands.getstatusoutput("DISPLAY=:0 xset q")
        if x_check[0] == 0:
            return True
        else:
            rospy.loginfo("X not available - sleeping for %s more seconds" % timeout)
            timeout -= 1
            rospy.sleep(1)


def dependency_available(server, port, name, timeout=None):
    """
    Wait for network service to appear. Provide addres, port and name.
    If timeout is set to none then wait forever.
    """
    import socket
    import errno
    from socket import error as socket_error

    s = socket.socket()
    if timeout:
        from time import time as now
        end = now() + timeout

    while True:
        try:
            if timeout:
                next_timeout = end - now()
                if next_timeout < 0:
                    return False
                else:
                    s.settimeout(next_timeout)

            s.connect((server, port))

        except socket_error as serr:
            # this exception occurs only if timeout is set
            if serr.errno == errno.ECONNREFUSED:
                rospy.logwarn("%s not yet available - waiting %s seconds more" % (name, next_timeout))
                rospy.sleep(1)
            else:
                rospy.logwarn("%s not available because: %s" % (name, serr))
                rospy.sleep(1)

        except socket.error, err:
            # catch timeout exception from underlying network library
            # this one is different from socket.timeout
            rospy.loginfo("%s not yet available - waiting %s secs more" % (name, next_timeout))
            rospy.sleep(1)
            if type(err.args) != tuple or err[0] != errno.ETIMEDOUT:
                raise
        else:
            s.close()
            rospy.loginfo("%s is available" % name)
            return True


def discover_host_from_url(url):
    from urlparse import urlparse
    data = urlparse(url)
    return data.hostname


def discover_port_from_url(url):
    from urlparse import urlparse
    data = urlparse(url)
    return data.port


def find_device(name):
    import os
    for device in os.listdir('/dev/input/'):
        device = '/dev/input/' + device
        if 'event' not in device:
            continue
        if check_device(device, name):
            return device
    # did not find an event device with the name provided
    return None


def check_device(device, name):
    import os
    from stat import ST_MODE
    from evdev import InputDevice
    if os.access(device, os.W_OK | os.R_OK):
        return (InputDevice(device).name == name)

    original_mode = os.stat(device)
    os.system('sudo chmod 0666 %s' % device)
    flag = (InputDevice(device).name == name)
    if not flag:
        os.chmod(device, original_mode)
    return flag


def is_valid_state(state):
    from lg_common.msg import ApplicationState
    return state == ApplicationState.HIDDEN or \
        state == ApplicationState.STOPPED or \
        state == ApplicationState.STARTED or \
        state == ApplicationState.SUSPENDED or \
        state == ApplicationState.VISIBLE


def make_soft_relaunch_callback(func, *args, **kwargs):
    """
    Creates a callback on the /soft_relaunch topic. The normal
    argument passed is an array of strings called 'groups.' Ros
    nodes can be put into groups like "sreetview" and "earth". The
    "all" group happens to all ros nodes.

    """
    from std_msgs.msg import String
    rospy.logdebug('creating callback %s' % kwargs.get('groups', 'no group'))

    def cb(msg):
        if msg.data == 'all':
            rospy.loginfo('calling callback for data: (%s) kwargs: (%s)' % (msg.data, kwargs))
            func(msg)
            return
        if 'groups' in kwargs and msg.data in kwargs['groups']:
            rospy.loginfo('calling callback for data: (%s) kwargs: (%s)' % (msg.data, kwargs))
            func(msg)
            return
    return rospy.Subscriber('/soft_relaunch', String, cb)


def get_nested_slot_value(slot, message):
    """
    Accepts a list a string with dots that represents slot and subslots
    needed to be traversed in order to get value of message's nested attribute
    Slot string gets converted to list of strings used to get a slot value from msg.
    Every subslot should be an element in the list. Example:

    For sensor_msg/Range:

    ---
    header:
      seq: 1414798
      stamp:
        secs: 1461247209
        nsecs: 611480951
      frame_id: ''
    radiation_type: 0
    field_of_view: 0.0
    min_range: 0.0
    max_range: 0.0
    range: 0.685800015926

    list of slots to get nsecs will look like:
        ['header', 'stamp', 'nsecs']

    Returns a dictionary with slots name and value e.g.:
        {'header.stamp.nsecs': 611480951}

    """
    slot_tree = slot.split('.')

    if len(slot_tree) == 1:
        return {slot: getattr(message, slot)}

    elif len(slot_tree) > 1:
        deserialized_msg = message
        for subslot in slot_tree:
            try:
                deserialized_msg = getattr(deserialized_msg, subslot)
            except AttributeError:
                if type(deserialized_msg) == str:
                    try:
                        # try to convert string to dict (works only for genericmessage)
                        deserialized_msg = json.loads(deserialized_msg)
                        deserialized_msg = deserialized_msg[subslot]
                    except KeyError:
                        msg = "Sublot %s does not exist in message: %s" % (subslot, deserialized_msg)
                        rospy.logerr(msg)
                    except ValueError:
                        msg = "Could not convert message '%s' to dict using subslot: '%s'" % (subslot, deserialized_msg)
                        rospy.logerr(msg)
                elif type(deserialized_msg) == dict:
                    try:
                        deserialized_msg = deserialized_msg[subslot]
                    except KeyError:
                        msg = "Could not get value for slot %s from message %s" % (subslot, deserialized_msg)
                        rospy.logerr(msg)
                else:
                    msg = "Could not get subslot value '%s' from message '%s'" % (subslot, deserialized_msg)
                    rospy.logerr(msg)

    return {slot: deserialized_msg}


def get_activity_config(scene, activity_name, window_viewport):
    """
    Returns configuration for the given activity on the given viewport in the given scene.

    Args:
        scene (interactivespaces_msgs.msg.GenericMessage)
        activity_name (str)
        window_viewport (str)

    Returns:
        dict: Configuration for the activity.
        None: Activity not present on this viewport.
    """
    import json
    scene = json.loads(scene.message)

    def is_activity_window(window):
        return window['activity'] == activity_name and \
            w['presentation_viewport'] == window_viewport

    try:
        windows = [w for w in scene['windows'] if is_activity_window(w)]
        activity_config = windows[0]['activity_config']
    except KeyError:
        return None
    except AttributeError:
        return None
    except IndexError:
        return None
    return activity_config


def check_www_dependency(should_depend, host, port, name, timeout):
    """
    Check if www dependency is available, or raise an exception
    """
    if should_depend:
        rospy.loginfo("Waiting for rosbridge to become available")
        if not dependency_available(host, port, name, timeout):
            msg = "Service: %s (%s:%s) hasn't become accessible within %s seconds" % (name, host, port, timeout)
            rospy.logfatal(msg)
            raise DependencyException(msg)
        else:
            rospy.loginfo("%s is online" % name)


def x_available_or_raise(timeout):
    """
    Checks if x is available, or will raise an error
    """
    if x_available(timeout):
        rospy.loginfo("X available")
    else:
        msg = "X server is not available"
        rospy.logfatal(msg)
        raise DependencyException(msg)


def browser_eligible_for_reuse(current_browser, future_browser):
    """
    type current_browser: ManagedAdhocBrowser
    type future_browser: AdhocBrowser.msg

    Compares two browsers and returns bool telling whether
    one browser can be updated to the other.
    It can't be updated if there's a difference in:
     - cmd args
     - extensions are different
     - user agent
     - binary
    """
    future_browser_extensions = [ext.name for ext in future_browser.extensions]
    future_browser_cmd_args = [arg.argument for arg in future_browser.command_line_args]

    return current_browser.user_agent == future_browser.user_agent and\
        current_browser.binary == future_browser.binary and\
        current_browser.extensions == future_browser_extensions and\
        current_browser.command_line_args == future_browser_cmd_args


def get_random_string(N=6):
    """
    Generate random string.
    """
    return ''.join(random.choice(string.ascii_uppercase) for _ in range(N))


def generate_hash(string, length=8, random_suffix=False):
    """
    Accept a string (typically AdhocBrowser.instance_hash()) and
    generate a unique hash with minimal collision

    This will ensure that objects have their unique instance
    hashes based on their representation (all attribs)

    random_suffix adds random string to the end of the hash.
    NB. random != unique it's still possible to get two equal hashes.
    """
    hash_str = base64.urlsafe_b64encode(hashlib.sha1(string).digest())[0:(length - 1)].replace('_', '')

    if random_suffix:
        return hash_str + "_" + get_random_string()
    else:
        return hash_str


def handle_initial_state(call_back):
    """
    Query for initial state from state service and run
    the call back with that state if available
    """
    # commenting out for now
    try:
        rospy.wait_for_service('/initial_state', 15)
    except:
        rospy.logerr("This system does not support initial state setting")
        return

    from lg_common.srv import InitialUSCS, InitialUSCSResponse

    initial_state_service = rospy.ServiceProxy('/initial_state', InitialUSCS)

    state = initial_state_service.call()
    if state and state != InitialUSCSResponse():
        rospy.loginfo('got initial state: %s for callback %s' % (state.message, call_back))
        call_back(state)
    else:
        rospy.logwarn('Could not get valid initial state for callback %s')


def route_touch_to_viewports(windows, route_touch_key='route_touch'):
    """
    Iterates over windows and returns list of viewports from
    activity_config with route_touch set to true

    Args:
        windows (dict): Windows from a director scene.
        route_touch_key (str): name of the attrib for touch routing

    Returns:
        set(str): Accumulated set of viewports that should be receiving
            touch events.
    """
    import re

    active_touch_routes = []

    for window in windows:
        activity = window.get('activity')

        config = window.get('activity_config', {})
        if route_touch_key not in config:
            continue
        if config.get(route_touch_key) is not True:
            continue

        source = config.get('viewport', '')
        viewport = re.sub(r'^viewport:\/\/', '', source, count=1)
        if viewport == '':
            viewport = window.get('presentation_viewport', '')

        active_touch_routes.append(viewport)

    return set(active_touch_routes)


def wait_for_pub_sub_connections(network=[], sleep=1, timeout=10, num_connections=1):
    """
    iterates over network items (typically publishers and subscribers)
    to wait for all of them to have at least 1 connection
    """
    timeout = timeout
    actors_names = ','.join([ actor.name for actor in network ])

    for interval in xrange(0, timeout):
        if all_actors_connected(network, num_connections):
            rospy.loginfo("All actors connected - ready to handle initial state")
            rospy.sleep(1)
            return True
        else:
            rospy.logwarn("Waiting for topics (%s) to become connected" % actors_names)
            rospy.sleep(sleep)
            timeout -= 1

    actors_names_connections = ','.join([ actor.name + ":" + str(actor.get_num_connections()) for actor in network ])

    message = "Some publishers and subscribers didnt reach %s connection requirement in %s seconds" % (num_connections, timeout)
    rospy.logerr(message)
    raise PublisherSubscriberConnectionsException(message)


def all_actors_connected(actors=[], num_connections=1):
    """
    Returns True if all actors have at least `num_connections` number of connections.

    Useful with methods that need to check if pub sub network is connected
    """
    for actor in actors:
        rospy.loginfo("Actor: %s num connections: %s, limit %s" % (actor.name, actor.get_num_connections(), num_connections))
        if actor.get_num_connections() < num_connections:
            return False
        else:
            rospy.logwarn("Actor: %s reached required number of connections: %s" % (actor.name, num_connections))

    return True
