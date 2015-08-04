"""Handlers for KMLSync server.

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
    asset_slug=foo,bar,baz
- for every asset not loaded on GE we need to create it by "<Create></Create>" section and delete by "Delete"
  section - those elements need to be children of "Update" element
- example networklinkupdate: https://github.com/EndPointCorp/lg_ros_nodes/issues/2#issuecomment-116727990

Special case - tours:
- when a tour KML is loaded - GE will try to reach KMLSyncServer's URL to send "playtour" command to itself (weird xD)
- Application needs to listen on /query.html link and accept GE's requests like:
    - /query.html?query=playtour=taipei (http://pastebin.com/3M5xHj0g)

    the query argument "playtour=taipei" needs to be forwarded to queryfile ROS node so GE gets a command to go
    to certain position

asset example: 'http://lg-head:8060/media/blah.kml'
cookie example: asset_slug=http___lg-head_8060_media_blah_kml

State data structure:
{ "right_one" : { "cookie": '', "assets": ['http://lg-head:8060/media/blah.kml] }
"""

import json
import rospy

import time
import urllib2
import threading
import xml.etree.ElementTree as ET

from xml.dom import minidom
from lg_earth.srv import KmlState, PlaytourQuery
from xml.sax.saxutils import unescape, escape
from lg_common.helpers import escape_asset_url, generate_cookie, write_log_to_file
from std_msgs.msg import String
from Queue import Queue
import tornado.web
import collections
from tornado import gen


def get_kml_root():
    """Get headers of KML file - shared by all kml generation methods."""
    kml_root = ET.Element('kml', attrib={})
    kml_root.attrib['xmlns'] = 'http://www.opengis.net/kml/2.2'
    kml_root.attrib['xmlns:gx'] = 'http://www.google.com/kml/ext/2.2'
    kml_root.attrib['xmlns:kml'] = 'http://www.opengis.net/kml/2.2'
    kml_root.attrib['xmlns:atom'] = 'http://www.w3.org/2005/Atom'
    return kml_root

class KmlMasterHandler(tornado.web.RequestHandler):

    def get(self):
        """Serve the master.kml which is updated by NLC."""
        rospy.loginfo("Got master.kml GET request")
        kml_root = get_kml_root()
        kml_document = ET.SubElement(kml_root, 'Document')
        kml_document.attrib['id'] = 'master'
        kml_reparsed = minidom.parseString(ET.tostring(kml_root))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        self.finish(kml_content)


class KmlUpdateHandler(tornado.web.RequestHandler):
    counter = 0
    TIMEOUT = 10
    global_queue = {}
    lock = threading.Lock()

    @classmethod
    def add_to_global_queue(cls, reference, unique_id):
        cls.global_queue[unique_id] = reference
        write_log_to_file("%s" % cls.global_queue)

    @classmethod
    def get_unique_id(cls):
        with cls.lock:
            unique_id = cls.counter
            cls.counter += 1
            write_log_to_file("Counter Value: %d" % unique_id)
        return unique_id

    @classmethod
    def finish_all_requests(cls):
        for req in cls.global_queue.itervalues():
            req.get(True)
        cls.global_queue = {}

    @classmethod
    def get_scene_msg(cls, msg):
        try:
            cls.finish_all_requests()
        except Exception as e:
            write_log_to_file("Exception saving scene"+str(e))
            pass

    def initialize(self):
        self.asset_service = self.application.asset_service
        self.__class__.timeout = self.application.request_timeout

    @gen.coroutine
    def get(self, second_time=False):
        """
        Return XML with latest list of assets for specific window_slug
            - get window slug and calculate difference between loaded assets and the desired state
            - create the KML and return it only if cookie was different and the window slug came in the request
        """
        rospy.loginfo("Got network_link_update.kml GET request with params: %s" % self.request.query_arguments)
        window_slug = self.get_query_argument('window_slug', default=None)
        incoming_cookie_string = ''

        incoming_cookie_string = self.get_query_argument('asset_slug', default='')

        rospy.loginfo("Got network_link_update GET request for slug: %s with cookie: %s" % (window_slug, incoming_cookie_string))

        if window_slug:
            try:
                assets = self._get_assets(window_slug)
            except Exception as e:
                rospy.logerr('Failed to get assets for {}: {}'.format(
                    window_slug,
                    e.message
                ))
                # Always return a valid KML or Earth will stop requesting updates
                self.finish(get_kml_root())
                return
            assets_to_delete = self._get_assets_to_delete(incoming_cookie_string, assets)
            assets_to_create = self._get_assets_to_create(incoming_cookie_string, assets)
            if (assets_to_delete or assets_to_create) or second_time:
                self.finish(self._get_kml_for_networklink_update(assets_to_delete, assets_to_create, assets))
            else:
                self.unique_id = unique_id = KmlUpdateHandler.get_unique_id()
                KmlUpdateHandler.add_to_global_queue(self, self.unique_id)
                yield gen.sleep(KmlUpdateHandler.timeout)
                if self.unique_id in KmlUpdateHandler.global_queue:
                    del KmlUpdateHandler.global_queue[self.unique_id]
                    assets_to_delete = self._get_assets_to_delete(incoming_cookie_string, assets)
                    assets_to_create = self._get_assets_to_create(incoming_cookie_string, assets)
                    #write_to_log("create %s\ndelete %s" % (assets_to_create, assets_to_delete))
                    self.finish(self._get_kml_for_networklink_update(assets_to_delete, assets_to_create, assets))
        else:
            self.set_status(400, "No window slug provided")
            self.finish("400 Bad Request: No window slug provided")


    def _get_kml_for_networklink_update(self, assets_to_delete, assets_to_create, assets):
        """ Generate static part of NetworkLinkUpdate xml"""
        kml_root = get_kml_root()
        kml_networklink = ET.SubElement(kml_root, 'NetworkLinkControl')
        kml_min_refresh_period = ET.SubElement(kml_networklink, 'minRefreshPeriod').text = '1'
        kml_max_session_length = ET.SubElement(kml_networklink, 'maxSessionLength').text = '-1'
        cookie_cdata_string = "<![CDATA[%s]]>" % self._get_full_cookie(assets)
        kml_cookies = ET.SubElement(kml_networklink, 'cookie').text = escape(cookie_cdata_string)
        kml_update = ET.SubElement(kml_networklink, 'Update')
        kml_target_href = ET.SubElement(kml_update, 'targetHref').text = self.request.protocol + '://' + self.request.host + '/master.kml'
        kml_create_assets = self._get_kml_for_create_assets(assets_to_create, kml_update)
        kml_delete_assets = self._get_kml_for_delete_assets(assets_to_delete, kml_update)

        kml_reparsed = minidom.parseString(unescape(ET.tostring(kml_root)))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        return unescape(kml_content)

    def _get_assets_to_delete(self, incoming_cookie_string, assets):
        """
        Calculate the difference between assets loaded on GE client and those expected to be loaded by server.
        Return a list of slugs to be deleted from client e.g. ['blah_kml', 'zomg_kml']
        param incoming_cookie_string: str
            e.g. 'blah_kml,zomg_kml'
        param assets: list
            list of assets for the request window_slug
        rtype: list
            e.g. ['blah_kml']
        """
        server_slugs_list = self._get_server_slugs_state(assets)
        client_slugs_list = self._get_client_slugs_state(incoming_cookie_string)
        ret = list(set(client_slugs_list) - set(server_slugs_list))
        rospy.logdebug("Got the assets to delete as: %s" % ret)
        return ret

    def _get_assets(self, window_slug):
        return self.asset_service(window_slug).assets

    def _get_assets_to_create(self, incoming_cookie_string, assets):
        """
        Calculate the difference between assets loaded on GE client and those expected to be loaded by server.
        Return a list of slugs to be created in GE client e.g. ['blah_kml', 'zomg_kml']
        param incoming_cookie_string: str
            e.g. 'blah_kml,zomg_kml'
        param assets: list
            list of assets for the request window_slug
        rtype: list
            e.g. ['http://lg-head:8060/blah.kml', 'http://lg-head:8060/zomg.kml']
        """

        urls_to_create = []
        client_state_slugs = self._get_client_slugs_state(incoming_cookie_string)

        for url in assets:
            if escape_asset_url(url) in client_state_slugs:
                continue
            urls_to_create.append(url)

        return urls_to_create

    def _get_client_slugs_state(self, incoming_cookie_string):
        """
        Return list of slugs submitted by client in a cookie string
        param incoming_cookie_string: str
            (e.g. 'blah_kml,zomg_foo_bar_kml')
        rtype: list
            e.g. ['blah_kml', 'zomg_foo_bar_kml']
        """
        if not incoming_cookie_string:
            return []
        return [z for z in incoming_cookie_string.split(',')]

    def _get_cookie(self, assets):
        """
        Return comma separated list of slugs e.g. 'asd_kml,blah_kml'
        rtype: str
        """
        return generate_cookie(assets)

    def _get_server_slugs_state(self, assets):
        """
        Return a list of slugs that server expects to be loaded
        """
        try:
            cookie = self._get_cookie(assets)
            return [z for z in cookie.split(',')]
        except AttributeError, e:
            return []

    def _get_full_cookie(self, assets):
        """return the cookie prepended by asset_slug= only if cookie is not blank"""
        cookie = self._get_cookie(assets)
        if cookie != '':
            cookie = 'asset_slug=' + cookie
        return cookie

    def _get_kml_for_create_assets(self, assets_to_create, parent):
        """
        Generate a networklinkupdate xml 'Create' section with assets_to_create (if any)
        """
        if assets_to_create:
            kml_create = ET.SubElement(parent, 'Create')
            kml_document = ET.SubElement(kml_create, 'Document', {'targetId': 'master'})
            for asset in assets_to_create:
                new_asset = ET.SubElement(kml_document, 'NetworkLink', {'id': escape_asset_url(asset)})
                ET.SubElement(new_asset, 'name').text = escape_asset_url(asset)
                link = ET.SubElement(new_asset, 'Link')
                ET.SubElement(link, 'href').text = asset
            return kml_create #not sure if this is needed since we're already building on the xml tree...
        return parent

    def _get_kml_for_delete_assets(self, assets_to_delete, parent):
        """
        Generate a networklinkupdate xml 'Delete' section with assets_to_delete (if any)
        """
        if assets_to_delete:
            kml_delete = ET.SubElement(parent, 'Delete')
            for asset in assets_to_delete:
                ET.SubElement(kml_delete, 'NetworkLink', {'targetId': escape_asset_url(asset)})
            return kml_delete


class KmlQueryHandler(tornado.web.RequestHandler):
    def initialize(self):
        self.playtour = self.application.playtour
        self.playtour_service = self.application.playtour_service

    def get(self):
        """
        Publish play tour message on the topic /earth/query/tour
        """
        query_string = self.get_query_argument('query', default='')
        try:
            while not rospy.is_shutdown():
                tour_string = query_string.split('=')[1]
                tour_string = urllib2.unquote(tour_string)
                self.playtour.tourname = str(tour_string)
                self.playtour_service(tour_string)
                self.finish("OK")
            self.set_status(503, "Server shutting down")
            self.finish("Server shutting down")
        except IndexError as e:
            rospy.logerr("Got the wrong query string %s" % e)
            self.set_status(400, "Got the wrong query string")
            self.finish("Bad Request: Got a bad query string")

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
