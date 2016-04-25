import os
import sys
import json
import rospy
import urllib
import urlparse

from interactivespaces_msgs.msg import GenericMessage


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
        rospy.logwarn("Got non json message on AdhocBrowserDirectorBridge")
        rospy.logdebug("Message: %s" % message)
        raise e

    return ret


def extract_first_asset_from_director_message(message, activity_type, viewport):
    """
    Extracts **single** (first) asset and geometry for given activity and viewport e.g. all assets for browser or all KMLs for GE.
    Returns list of dictionaries containing adhoc browser metadata e.g.:
        [
        { "1": { "path": "https://www.youtube.com/watch?v=un8FAjXWOBY",
                 "height": 600,
                 "width":800,
                 "x_coord":100,
                 "y_coord":100 }},
        { "2": { "path": "https://www.youtube.com/watch?v=un8FAjXWOBY",
                 "height": 150,
                 "width": 300,
                 "x_coord": 100,
                 "y_coord": 100 }}]

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
            asset_object['path'] = window['assets'][0]
            asset_object['x_coord'] = window['x_coord']
            asset_object['y_coord'] = window['y_coord']
            asset_object['height'] = window['height']
            asset_object['width'] = window['width']
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
        else:
            previous_item = item
    return True


def rewrite_message_to_dict(message):
    """
    Converts any object with __slots__ to a dictionary
    """
    deserialized_message = {}
    slots = message.__slots__
    if slots.__class__ == str:
        slots = [slots]
    for slot in slots:
        deserialized_message[slot] = getattr(message, slot)
    return deserialized_message


def unpack_activity_sources(sources_string):
    """
    Unpacks ActivityTracker 'sources' param

    Sources string format:

    <topic_name>:<message_type>-<slot[.sub_slot.sub_sub_slot>]:<strategy>[-<value_min><value_max>]

    - topic_name: topic which ActivitySource will listen for incoming messages
    - message_type: type of message on the given topic
    - slot - for complex messages you may define one attribute of the message that
    will be considered during checks. For nested attributes, delimit slots by dots.
    - strategy - delta, value, activity, average and count. For "value" strategy you may define min/max values.
    It will make sense only when slot was defined.

    Strategies info:
     - delta means that activity is generated only if messages are different. Makes
      sense with spacenav twist which emits 0,0,0 when no one's touching it
     - value is used for value tresholding - generate activity only when value of slot met min/max constraints
     - activity - triggers activity on **every** message
     - average - not used to activity tracking - passes information about what should be done with values
     - count - same as above

    Examples:

    - source string: '/touchscreen/touch:interactivespaces_msgs/GenericMessage:activity'

    result: source = { "topic": "/touchscreen/touch",
               "msg_type": "interactivespaces_msgs/GenericMessage",
               "strategy": "activity",
               "slot": None,
               "value_min": None,
               "value_max": None
             }

    - source string:'/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5':

    result: source = { "topic": "/proximity_sensor/distance",
               "msg_type": "sensor_msgs/Range",
               "strategy": "value",
               "slot": range,
               "value_min": 0,
               "value_max": 2.5
             }

    NOTE: min/max values is the range within which active == True

    - source string:'/proximity_sensor/distance:sensor_msgs/Range-range:average':

    result: source = { "topic": "/proximity_sensor/distance",
               "msg_type": "sensor_msgs/Range",
               "strategy": "average",
               "slot": range,
               "value_min": None,
               "value_max": None
             }

    To define multiple sources use semicolon e.g.:

    '/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5;/touchscreen/touch:interactivespaces_msgs/GenericMessage:delta'

    """

    sources = []
    bare_sources = sources_string.split(';')
    for source_string in bare_sources:
        single_source = {}
        source_fields = source_string.split(':')

        topic = source_fields[0]
        try:
            msg_type = source_fields[1].split('-')[0]
        except IndexError, e:
            msg_type = source_fields[1]

        try:
            slot = source_fields[1].split('-')[1]
        except IndexError, e:
            slot = None

        try:
            strategy = source_fields[2].split('-')[0]
        except IndexError, e:
            strategy = source_fields[2]

        try:
            values = source_fields[2].split('-')[1].split(',')
            value_min = values[0]
            value_max = values[1]
            rospy.loginfo("Detected values min: %s, max:%s for source_string: %s" % (value_min, value_max, source_string))
        except Exception, e:
            rospy.loginfo("Could not get value_min/value_max from sources for source_string: %s" % source_string)
            value_min = None
            value_max = None
        single_source['topic'] = topic
        single_source['msg_type'] = msg_type
        single_source['strategy'] = strategy
        single_source['slot'] = slot
        single_source['value_min'] = value_min
        single_source['value_max'] = value_max
        sources.append(single_source)
    return sources


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
    rospy.loginfo('creating callback %s' % kwargs.get('groups', 'no group'))

    def cb(msg):
        rospy.logerr('calling callback for data: (%s) kwargs: (%s)' % (msg.data, kwargs))
        if msg.data == 'all':
            rospy.loginfo('firing function for all...')
            func(msg)
            return
        if 'groups' in kwargs and msg.data in kwargs['groups']:
            rospy.loginfo('firing function for custom msg...')
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
        for slot_number in xrange(0, len(slot_tree)):
            if slot_number == 0:
                deserialized_msg = getattr(message, slot_tree[slot_number])
            else:
                deserialized_msg = getattr(deserialized_msg, slot_tree[slot_number])
        else:
            msg = "Wrong slot_tree provided: %s" % slot
            rospy.logerr(msg)
            raise SlotUnpackingException(msg)

    return {slot: deserialized_msg}
