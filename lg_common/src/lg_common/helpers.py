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

def unpack_activity_sources(sources):
    """
    unpacks ActivityTracker 'sources' param from '/spacenav/twist:geometry_msgs/Twist:delta' to:
    {/spacenav/twist': {'msg_type': 'geometry_msgs/Twist',
                        'strategy': 'delta'}}
    """
    try:
        {source.split(':')[0] : { 'msg_type': source.split(':')[1], 'strategy': source.split(':')[2]} for source in sources.split(',')}
    except Exception, e:
        exception_msg = "Could not unpack activity sources string because: %s" % e
        rospy.logerr(exception_msg)
        raise WrongActivityDefinition(exception_msg)
