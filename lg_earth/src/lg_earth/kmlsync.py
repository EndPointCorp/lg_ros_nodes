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

import urllib.request, urllib.error, urllib.parse
import threading
import xml.etree.ElementTree as ET

from xml.dom import minidom
from lg_msg_defs.srv import KmlState, PlaytourQuery
from xml.sax.saxutils import unescape, escape
from lg_common.helpers import escape_asset_url, generate_cookie
from std_msgs.msg import String
import tornado.web
from tornado import gen
from tornado.concurrent import Future
from tornado.ioloop import IOLoop
from lg_common.logger import get_logger
logger = get_logger('kmlsync')


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
        logger.debug("Got master.kml GET request")
        kml_root = get_kml_root()
        kml_document = ET.SubElement(kml_root, 'Document')
        kml_document.attrib['id'] = 'master'
        kml_reparsed = minidom.parseString(ET.tostring(kml_root))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        self.finish(kml_content.encode('utf8'))


class KmlUpdateHandler(tornado.web.RequestHandler):
    counter = 0
    timeout = 0
    deferred_requests = {}
    counter_lock = threading.Lock()
    defer_lock = threading.Lock()

    @classmethod
    def add_deferred_request(cls, reference, unique_id):
        with cls.defer_lock:
            cls.deferred_requests[unique_id] = reference

    @classmethod
    def get_unique_id(cls):
        with cls.counter_lock:
            unique_id = cls.counter
            cls.counter += 1
        return unique_id

    @classmethod
    def finish_all_requests(cls):
        with cls.defer_lock:
            for req in cls.deferred_requests.values():
                req.get(no_defer=True)
            cls.deferred_requests.clear()

    @classmethod
    def get_scene_msg(cls, msg):
        try:
            cls.finish_all_requests()
        except Exception as e:
            logger.error("Exception getting scene changes" + str(e))
            pass

    def non_blocking_sleep(self, duration):
        f = Future()
        IOLoop.current().call_later(duration, lambda: f.set_result(None))
        return f

    def initialize(self):
        self.asset_service = self.application.asset_service

    @gen.coroutine
    def get(self, no_defer=False):
        """
        Return XML with latest list of assets for specific window_slug
            - get window slug and calculate difference between loaded assets and the desired state
            - create the KML and return it only if cookie was different and the window slug came in the request
        """
        # Never defer if there's no timeout
        if KmlUpdateHandler.timeout <= 0:
            no_defer = True

        logger.debug("Got network_link_update.kml GET request with params: %s" % self.request.query_arguments)
        window_slug = self.get_query_argument('window_slug', default=None)
        incoming_cookie_string = self.get_query_argument('asset_slug', default='')

        logger.debug("Got network_link_update GET request for slug: %s with cookie: %s" % (window_slug, incoming_cookie_string))

        if not window_slug:
            self.set_status(400, "No window slug provided")
            self._finish_text("400 Bad Request: No window slug provided")
            return

        try:
            assets = self._get_assets(window_slug)
        except Exception:
            logger.exception('Failed to get assets for {}'.format(
                window_slug,
            ))
            # Always return a valid KML or Earth will stop requesting updates
            self._finish_text(get_kml_root())
            return

        assets_to_create, assets_to_delete = self._get_asset_changes(incoming_cookie_string, assets)
        if (assets_to_delete or assets_to_create) or no_defer:
            self._finish_text(self._get_kml_for_networklink_update(assets_to_delete, assets_to_create, assets))
            return

        self.unique_id = KmlUpdateHandler.get_unique_id()

        logger.debug("Request Counter: {}".format(self.unique_id))
        logger.debug("Deferred Requests: {}".format(KmlUpdateHandler.deferred_requests))

        KmlUpdateHandler.add_deferred_request(self, self.unique_id)
        yield self.non_blocking_sleep(KmlUpdateHandler.timeout)

        try:
            assets = self._get_assets(window_slug)
        except Exception as e:
            logger.error('Failed to get assets for {}: {}'.format(
                window_slug,
                e.message
            ))
            # Always return a valid KML or Earth will stop requesting updates
            self._finish_text(get_kml_root())
            return

        with KmlUpdateHandler.defer_lock:
            if self.unique_id not in KmlUpdateHandler.deferred_requests:
                return
            del KmlUpdateHandler.deferred_requests[self.unique_id]

            assets_to_create, assets_to_delete = self._get_asset_changes(incoming_cookie_string, assets)
            self._finish_text(self._get_kml_for_networklink_update(assets_to_delete, assets_to_create, assets))

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

        kml_reparsed = minidom.parseString(unescape(ET.tostring(kml_root).decode('utf-8')))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        return unescape(kml_content)

    def _get_assets(self, window_slug):
        return self.asset_service(window_slug).assets

    def _get_asset_changes(self, incoming_cookie_string, assets):
        """
        Shortcut to get all asset changes.
        param incoming_cookie_string: str
            e.g. 'blah_kml,zomg_kml'
        param assets: list
            list of assets for the request window_slug
        rtype: tuple
            combined assets to create, delete
        """
        return (
            self._get_assets_to_create(incoming_cookie_string, assets),
            self._get_assets_to_delete(incoming_cookie_string, assets),
        )

    def _finish_text(self, text):
        """Encode and finish request with provided text.
        param text: str
            text to send
        """
        self.finish(text.encode('utf8'))

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
        logger.debug("Got the assets to delete as: %s" % ret)
        return ret

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
        except AttributeError as e:
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
        if not assets_to_create:
            return

        kml_create = ET.SubElement(parent, 'Create')
        kml_document = ET.SubElement(kml_create, 'Document', {'targetId': 'master'})
        for asset in assets_to_create:
            new_asset = ET.SubElement(kml_document, 'NetworkLink', {'id': escape_asset_url(asset)})
            ET.SubElement(new_asset, 'name').text = escape_asset_url(asset)
            link = ET.SubElement(new_asset, 'Link')
            ET.SubElement(link, 'href').text = escape(asset)
        return kml_create

    def _get_kml_for_delete_assets(self, assets_to_delete, parent):
        """
        Generate a networklinkupdate xml 'Delete' section with assets_to_delete (if any)
        """
        if not assets_to_delete:
            return

        kml_delete = ET.SubElement(parent, 'Delete')
        for asset in assets_to_delete:
            ET.SubElement(kml_delete, 'NetworkLink', {'targetId': escape_asset_url(asset)})
        return kml_delete


class KmlQueryHandler(tornado.web.RequestHandler):
    def initialize(self):
        self.playtour_service = self.application.playtour_service
        self.planet_service = self.application.planet_service
        self.get_planet = self.application.get_planet

    def _wait_for_planet(self, planet):
        sleeptime = 0.1  # How long to sleep between iterations
        intervals = 50   # How many iterations to try before giving up
        interval_count = 0
        done = False

        while self.get_planet() != planet and interval_count < intervals:
            interval_count += 1
            rospy.sleep(sleeptime)

    def get(self):
        """
        Publish messages for the lg_earth/query service, in response to GET
        requests
        """
        query_string = self.get_query_argument('query', default='')

        if rospy.is_shutdown():
            self.set_status(503, "Server shutting down")
            self._finish_text("Server shutting down")
            return

        try:
            for op in query_string.split(','):
                command, value = op.split('=')
                value = urllib.parse.unquote(value)

                if command == 'playtour':
                    logger.info("Playing tour %s" % value)
                    self.playtour_service(value)

                elif command == 'planet':
                    logger.info("Switching to planet %s" % value)
                    self.planet_service(value)
                    self._wait_for_planet(value)

                elif command == 'wait':
                    try:
                        rospy.sleep(float(value))
                    except ValueError as e:
                        logger.error(
                            "Failed to convert %s to a float, while trying to sleep" %
                            value)

                else:
                    logger.error("unknown query command: %s" % command)

            self._finish_text("OK")

        except (IndexError, ValueError) as e:
            logger.error("Failed to split/parse query string: {}".format(query_string))
            self.set_status(400, "Got a bad query string")
            self._finish_text("Bad Request: Got a bad query string")
            return

    def _finish_text(self, text):
        """Encode and finish request with provided text.
        param text: str
            text to send
        """
        self.finish(text.encode('utf8'))

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
