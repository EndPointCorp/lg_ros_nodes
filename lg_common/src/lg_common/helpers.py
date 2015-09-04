import sys
import json
import rospy
import urllib
import urlparse

class WrongActivityDefinition(Exception):
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

    #4th index is the query params position
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
    cookie = (',').join([ escape_asset_url(asset) for asset in assets ])
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

    try:
        message = json.loads(message.message)
    except (ValueError, SyntaxErorr) as e:
        rospy.logwarn("Got non json message on AdhocBrowserDirectorBridge for viewport %s" % viewport)
        rospy.logdebug("Message: %s" % message)
        return []

    rospy.logdebug("Message: %s, activity_type: %s, viewport: %s" % (message, activity_type, viewport))
    assets = []
    for window in message['windows']:
        if (window['activity'] == activity_type) and (window['presentation_viewport'] == viewport):
            asset_object = {}
            asset_object['path'] = window['assets'][0]
            asset_object['x_coord'] = window['x_coord']
            asset_object['y_coord'] = window['y_coord']
            asset_object['height'] = window['height']
            asset_object['width'] = window['width']
            assets.append(asset_object)
        else:
            rospy.logdebug("Message was not directed at activity %s on viewport %s" % (window['activity'], window['presentation_viewport']))

    rospy.logdebug("Returning assets: %s" % assets)
    return assets

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
    for slot in slots:
        deserialized_message[slot] = getattr(message, slot)
    return deserialized_message

def unpack_activity_sources(sources_string):
    """
    Unpacks ActivityTracker 'sources' param

    Sources string format:

    <topic_name>/<message_type>[(<slot.sub_slot.sub_sub_slot>)]/<strategy>[(<value_min><value_max>)]

    - topic_name: topic which ActivitySource will listen for incoming messages
    - message_type: type of message on the given topic
    - slot - for complex messages you may define one attribute of the message that
    will be considered during checks. For nested attributes, delimit slots by dots.
    - strategy - delta, value or activity. For "value" strategy you may define min/max values.
    It will make sense only when slot was defined.

    Examples:

    source string: '/touchscreen/touch:interactivespaces_msgs/GenericMessage:delta'

    result: source = { "topic": "/touchscreen/touch",
               "msg_type": "interactivespaces_msgs/GenericMessage",
               "strategy": "activity",
               "slot": None,
               "value_min": None,
               "value_max": None
             }

    source string:'/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5':

    result: source = { "topic": "/touchscreen/touch",
               "msg_type": "interactivespaces_msgs/GenericMessage",
               "strategy": "activity",
               "slot": None,
               "value_min": 0,
               "value_max": 2.5
             }

    NOTE: min/max values is the range within which active == True

    To define multiple sources use semicolon e.g.:

    '/proximity_sensor/distance:sensor_msgs/Range-range:value-0,2.5;/touchscreen/touch:interactivespaces_msgs/GenericMessage:delta'

    """

    try:
        sources = []
        bare_sources = sources_string.split(';')
        for source_string in bare_sources:
            single_source = {}
            source_fields = source_string.split(':')

            topic = source_fields[0]
            msg_type = source_fields[1].split('-')[0]
            try:
                slot = source_fields[1].split('-')[1]
            except IndexError,e:
                slot = None

            strategy = source_fields[2].split('-')[0]
            try:
                values = source_fields[2].split('-')[1]
            except IndexError, e:
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
    except Exception, e:
        exception_msg = "Could not unpack activity sources string because: %s" % e
        rospy.logerr(exception_msg)
        raise WrongActivityDefinition(exception_msg)

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
