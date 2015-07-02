import json
import rospy
import xml.etree.ElementTree as ET
import requests

from xml.dom import minidom
from flask import Flask, request
from lg_common import SceneListener
from xml.sax.saxutils import unescape, escape
from flask.ext.classy import FlaskView, route
from lg_common.helpers import escape_asset_url
from lg_common.helpers import write_log_to_file
from interactivespaces_msgs.msg import GenericMessage


class KMLSyncServer(FlaskView):
    """ Run a KMLsync HTTP server
        Overview:
        - GE loads 'myplaces.kml' after it starts (lg_earth should create it)
        - myplaces.kml contains networklinks to master.kml and to regular
          updates
        - master.kml is a container for content
        - network_link_update.kml returns commands to change the contents of
          the container

        Example scenario:
        - GE loads myplaces (contains networklinks to master and networklinkupdate.kml with viewport name)
        - master.kml returns ampty document
        - networklinkupdate will return list of commands with empty cookiestring

        <director message loads smthn>

        - earth hits networklinkupdate.kml - we see the viewport - it should
          have something loaded now - we check the cookiestring (which is empty)
        - since it doesnt have anything loaded, we tell GE to load all the assets
        - we have a list of slugs and URLs from the director that we want to load
        - networklinkupdate will return a different cookie string with a list of the slugs e.g.
            asset_slug=foo&asset_slug=bar&asset_slug=baz
        - for every asset not loaded on GE we need to create it by "<Create></Create>" section and delete by "Delete"
          section - those elements need to be children of "Update" element
        - example networklinkupdate: https://github.com/EndPointCorp/lg_ros_nodes/issues/2#issuecomment-116727990

        Special case - tours:
        - when a tour KML is loaded - GE will try to reach KMLSyncServer's URL to send "playtour" command to itself (weird xD)
        - KMLSyncServer needs to listen on /query.html link and accept GE's requests like:
            - /query.html?query=playtour=taipei (http://pastebin.com/3M5xHj0g)

            the query argument "playtour=taipei" needs to be forwarded to queryfile ROS node so GE gets a command to go
            to certain position

        asset example: 'http://lg-head:8060/media/blah.kml'
        cookie example: asset_slug=http___lg-head_8060_media_blah_kml

        State data structure:
        { "right_one" : { "cookie": '', "assets": ['http://lg-head:8060/media/blah.kml] }
    """

    def __init__(self):
        self.host = rospy.get_param('~kmlsync_listen_host', '127.0.0.1')
        self.port = rospy.get_param('~kmlsync_listen_port', 8765)

        self.sub = rospy.Subscriber('/director/scene', GenericMessage, self._scene_listener_callback)

        write_log_to_file("Initialized scene listener (%s)" % self.__repr__)

        self.assets_state = {}

        rospy.on_shutdown(self._shutdown_hook)

    @route('/shutdown')
    def shutdown(self):
        self.shutdown_server()
        return "Flask server shutting down at %s" % self.__repr__

    @route('/master.kml')
    def master_kml(self):
        rospy.loginfo("Got master.kml GET request")
        write_log_to_file("master.kml repr of self.assets_state => %s" % self.assets_state.__repr__)
        kml_root = self._get_kml_root()
        kml_document = ET.SubElement(kml_root, 'Document')
        kml_document.attrib['id'] = 'master'
        kml_reparsed = minidom.parseString(ET.tostring(kml_root))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        return kml_content

    @route('/network_link_update.kml')
    def network_link_update(self):
        """
        Return XML with latest list of assets for specific window_slug
            - get window slug and calculate difference between loaded assets and the desired state
            - create the KML and return it only if cookie was different and the window slug came in the request
        """
        rospy.loginfo("Got network_link_update.kml GET request with params: %s" % request.args)
        window_slug = request.args.get('window_slug', None)
        incoming_cookie_string = ''

        for argument, value in request.args.iteritems():
            if argument == "asset_slug":
                incoming_cookie_string += argument + "=" + value

        rospy.loginfo("Got network_link_update GET request for slug: %s with cookie: %s" % (window_slug, incoming_cookie_string))

        if window_slug:
            assets_to_delete = self._get_assets_to_delete(incoming_cookie_string, window_slug)
            assets_to_create = self._get_assets_to_create(incoming_cookie_string, window_slug)
            return self._get_kml_for_networklink_update(assets_to_delete, assets_to_create, window_slug)
        else:
            return '', 400

    def shutdown_server(self):
        func = request.environ.get('werkzeug.server.shutdown')
        rospy.loginfo("Shutting down flask server")
        write_log_to_file("Shutting down flask server at %s" % self.__repr__)
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()

    def _shutdown_hook(self):
        write_log_to_file("Making request inside shutdown_hook at %s" % self.__repr__)
        try:
            requests.get('http://' + self.host + ':' + str(self.port) + '/shutdown')
        except ConnectionError, e:
            rospy.logerr("Couldnt execute shutdown hook")
            write_log_to_file("Couldnt execute shutdown hook")

    def _scene_listener_callback(self, scene):
        """
        Scene listener callback for KMLSyncServer

        - unpack director message only if the activity attrib == 'ge clients'
        - return the state of the assets for the webserver
         - return the "assets" list per viewport
        {
         "viewport_name1": [ "asset1", "asset2" ],
         "viewport_name2": [ "asset3", "asset4" ]
        }
        """
        write_log_to_file("Got a message on _scene_listener_callback" )
        assert scene.type == 'json'

        message = json.loads(scene.message)

        if 'windows' not in message:
            rospy.loginfo("received invalid message... ignoring")
            write_log_to_file("invalid message being ignored %s \n" % scene)
            return

        for window in message['windows']:
            if window['activity'] == 'earth':
                #TODO fix this here, used to be self.http_server which doesn't exist,
                # so how are we supposed to _set_assets now?
                self._set_assets(window['presentation_viewport'], window['assets'])

    def _set_assets(self, viewport, assets):
        write_log_to_file("Begining assets setting")
        try:
            write_log_to_file("Trying to make an assert about %s and %s" % (viewport, assets))
            assert isinstance(viewport, unicode), "Viewport name was invalid"
            assert isinstance(assets, list), "Director message did not contain window list"
        except AssertionError, e:
            rospy.loginfo("Director message error %s" % e)
            write_log_to_file("Director message error")

        self.assets_state[viewport] = {'assets': assets,
                                       'cookie': self._generate_cookie(assets) }

        write_log_to_file("Here's the full state: %s (%s)" % (self.assets_state, self.assets_state.__repr__))

    """ Private methods below """

    def _generate_cookie(self, assets):
        cookie = ('&').join([ 'asset_slug=' + escape_asset_url(asset) for asset in assets ])
        rospy.logdebug("Generated cookie = %s after new state was set" % cookie)
        return cookie

    def _get_kml_root(self):
        kml_root = ET.Element('kml', attrib={})
        kml_root.attrib['xmlns'] = 'http://www.opengis.net/kml/2.2'
        kml_root.attrib['xmlns:gx'] = 'http://www.google.com/kml/ext/2.2'
        kml_root.attrib['xmlns:kml'] = 'http://www.opengis.net/kml/2.2'
        kml_root.attrib['xmlns:atom'] = 'http://www.w3.org/2005/Atom'
        return kml_root

    def _get_server_slugs_state(self, window_slug):
        try:
            return [ z.replace("asset_slug=", "") for z in self.assets_state.get(window_slug, {'cookie': ''})['cookie'].split('&')]
        except AttributeError, e:
            return []

    def _get_client_slugs_state(self, incoming_cookie_string):
        return [ z.replace("asset_slug=", "") for z in incoming_cookie_string.split('&')]

    def _get_assets_to_delete(self, incoming_cookie_string, window_slug):
        server_slugs_list = self._get_server_slugs_state(window_slug)
        client_slugs_list = self._get_client_slugs_state(incoming_cookie_string)
        ret = list(set(server_slugs_list) - set(client_slugs_list))
        rospy.logdebug("Got the assets to delete as: %s" % ret)
        return ret

    def _get_assets_to_create(self, incoming_cookie_string,  window_slug):
        """
        For every url on server state list,
        take url's slug and check if it's loaded on client (according to client's cookie string)
        if it's not there - tell client to load it
        """

        urls_to_create = []
        client_state_slugs = self._get_client_slugs_state(incoming_cookie_string)

        for url in self.assets_state.get(window_slug, {'assets': []})['assets']:
            if escape_asset_url(url) in client_state_slugs:
                continue
            urls_to_create.append(url)

        return urls_to_create

    def _get_kml_for_networklink_update(self, assets_to_delete, assets_to_create, window_slug):
        kml_root = self._get_kml_root()
        kml_networklink = ET.SubElement(kml_root, 'NetworkLinkControl')
        kml_min_refresh_period = ET.SubElement(kml_networklink, 'minRefreshPeriod').text = '1'
        kml_max_session_length = ET.SubElement(kml_networklink, 'maxSessionLength').text = '-1'
        cookie_cdata_string = "<![CDATA[%s]]>" % self.assets_state.get(window_slug, {'cookie': '', 'assets': []})['cookie']
        write_log_to_file("Retrieved cookie: %s from state: %s with window_slug: %s (%s)" % (cookie_cdata_string, self.assets_state, window_slug, self.assets_state.__repr__))
        kml_cookies = ET.SubElement(kml_networklink, 'cookie').text = escape(cookie_cdata_string)
        kml_update = ET.SubElement(kml_networklink, 'Update')
        kml_target_href = ET.SubElement(kml_update, 'targetHref').text = 'http://' + self.host + ':' + str(self.port) + '/master.kml'
        kml_create_assets = self._get_kml_for_create_assets(assets_to_create, kml_update)
        kml_delete_assets = self._get_kml_for_delete_assets(assets_to_delete, kml_update)

        kml_reparsed = minidom.parseString(unescape(ET.tostring(kml_root)))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        return unescape(kml_content)

    def _get_kml_for_create_assets(self, assets_to_create, parent):
        kml_create = ET.SubElement(parent, 'Create')
        kml_document = ET.SubElement(kml_create, 'Document', {'targetId': 'master'})
        for asset in assets_to_create:
            new_asset = ET.SubElement(kml_document, 'NetworkLink', {'id': escape_asset_url(asset)})
            ET.SubElement(new_asset, 'name').text = escape_asset_url(asset)
            link = ET.SubElement(new_asset, 'Link')
            ET.SubElement(link, 'href').text = asset
        return kml_create #not sure if this is needed since we're already building on the xml tree...

    def _get_kml_for_delete_assets(self, assets_to_delete, parent):
        if assets_to_delete:
            kml_delete = ET.SubElement(parent, 'Delete')
            for asset in assets_to_delete:
                ET.SubElement(kml_delete, 'NetworkLink', {'targetId': escape_asset_url(asset)})
            return kml_delete


if __name__ == "__main__":
    print "Dont call me directly please"
