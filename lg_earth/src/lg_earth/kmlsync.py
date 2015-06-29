import rospy
import xml.etree.ElementTree as ET
from lg_common import SceneListener
from lg_common import webapp
from flask import render_template
from xml.dom import minidom


class KMLFlaskApp:
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
    """

    def __init__(self, port, host):
        self.host = host
        self.port = port
        self.app = Flask(__name__)

        self.assets_state = {}

    def run(self):
        webapp.ros_flask_spin(self.app, host=self.host, port=self.port)

    def set_assets(self, viewport, assets):
        try:
            assert isinstance(viewport, str), "Viewport name was invalid"
            assert isinstance(assets, list), "Director message did not contain window list"
        except AssertionError, e:
            rospy.logerror("Director message error %s" % e)

        self.assets_state[viewport] = assets

    def _get_kml_xml_root(self):
        kml_root = ET.Element('kml', attrib={})
        kml_root.attrib['xmlns'] = 'http://www.opengis.net/kml/2.2'
        kml_root.attrib['xmlns:gx'] = 'http://www.google.com/kml/ext/2.2'
        kml_root.attrib['xmlns:kml'] = 'http://www.opengis.net/kml/2.2'
        kml_root.attrib['xmlns:atom'] = 'http://www.w3.org/2005/Atom'
        return kml_root

    @app.route('/master.kml')
    def master_kml(self):
        rospy.loginfo("Got master.kml GET request")
        kml_root = self._get_kml_xml_root()
        kml_document = ET.SubElement(kml_root, 'Document')
        kml_document.attrib['id'] = 'master'
        kml_reparsed = minidom.parseString(ET.tostring(kml_root))
        kml_content = kml_reparsed.toprettyxml(indent='\t')
        return kml_content

    @app.route('/network_link_update.kml')
    def networklink_update(self):
        rospy.debuginfo("Got network_link_update.kml GET request with params: %s" % request.args)
        window_slug = request.args.get('window_slug
        kml_root = self._get_kml_xml_root()
        
        cookie_string = request.args.get('key', '')
        window_slug = request.args.get('window_slug', '')



class KMLSyncServer:
    def __init__(self,
                 host=None,
                 port=None):

        self.http_server = KMLFlaskApp(host=host, port=port).run()
        self.scene_listener = SceneListener(self.scene_listener_callback)

    def scene_listener_callback(self, scene):
        """
        - unpack director message only if the slug=='ge clients'
        - return the state of the assets for the webserver
         - return the "assets" list per viewport
        {
         "viewport_name1": [ "asset1", "asset2" ],
         "viewport_name2": [ "asset3", "asset4" ]
        }
        """
        for window in scene['windows']:
            if activity == 'earth':
                self.http_server.set_assets(window['presentation_viewport'], window['assets'])

    def run(self):
        self.http_server.run()
