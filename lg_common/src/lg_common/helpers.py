import rospy
import urllib
import urlparse

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
