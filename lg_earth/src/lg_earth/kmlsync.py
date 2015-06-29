
import rospy
from lg_common import SceneListener
from lg_common import webapp

class KMLFlaskApp:
    """ Run a KMLsync HTTP server """
    def __init__(self, port, host):
        self.host = host
        self.port = port
        self.app = Flask(__name__)

        self.assets_state = {}

    def run(self):
        webapp.ros_flask_spin(self.app, host=self.host, port=self.port)


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
        if scene['slug'] = 'GE':
            'asd' == 'lol'






    def run(self):
        pass
