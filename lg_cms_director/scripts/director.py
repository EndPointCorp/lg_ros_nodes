#!/usr/bin/env python

# This is heavily influenced by Pulsar's examples,
# especially `chat`, `tweets`, and `philosophers`.
# http://pythonhosted.org/pulsar/tutorials/index.html

import os
import sys
import json
import rospy
import signal


from pulsar.apps import wsgi, ws, Application, MultiApp
from pulsar.apps.data import create_store
from pulsar.apps.ds import pulsards_url
from pulsar.apps.http import HttpClient
from pulsar import command, task, coroutine_return

from interactivespaces_msgs.msg import GenericMessage


api_url = rospy.get_param(
    '~director_api_url',
    os.getenv('DIRECTOR_API_URL', 'http://localhost:8034')
)
DIRECTOR_API_URL = api_url

rospy.loginfo("Using DIRECTOR_API_URL = %s" % api_url)

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')


def _check_registration(e):
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
    """\
    Periodically check the health of this node on the master.
    """
    rospy.Timer(rospy.Duration(interval), _check_registration)


def next_scene_uri(presentation, scene):
    """\
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


class Resetter():
    """\
    A configured function called to reset the Scene Timer.
    """
    def __init__(self, worker, function):
        self.worker = worker
        self.function = function

    def __call__(self, channel, message):
        # Make sure this is actually a new Scene, not a Presentation.
        if channel != 'scene':
            return  # nevermind

        # Parse the new Scene for a duration.
        try:
            duration = json.loads(message)['duration']
        except KeyError:
            rospy.logerror("couldn't find a duration in this scene.")
            return

        # Call the function we were created with.
        self.function(self.worker, duration)


class DirectorWS(ws.WS):
    """\
    Add additional features to this WebSocket handler to connect it
    with our state data store, both pub/sub and key/value.
    """
    # This is very similar to pulsar.ws.PubSubWS

    def __init__(self, store, channel):
        self.store = store
        # Create and subscribe to the data store's publish-subscribe handler.
        self.pubsub = self.store.pubsub()
        self.channel = channel
        self.pubsub.subscribe(self.channel)
        # Create a client connection to the data store's key/value store.
        self.client = self.store.client()

    def publish_set(self, channel, message):
        # Both set a key and publish a message on the data store.
        self.pubsub.publish(channel, message)
        self.client.set(self.channel, message)

    def on_message(self, websocket, message):
        if message:
            # When a message arrives on a WebSocket, publish to the store.
            self.publish_set(self.channel, message)

    def on_open(self, websocket):  # When a new connection is established
        # Add a client that writes new pub/sub messages back to the websocket.
        self.pubsub.add_client(ws.PubSubClient(websocket, self.channel))
        # Fetch the most recent message and send it to the client.
        last_message = yield self.client.get(self.channel)
        websocket.write(last_message)


class ProxyRouter(wsgi.Router):
    """\
    WSGI Middleware to proxy HTTP GET requests to the content repository.
    """
    def __init__(self, rule, root):
        super(ProxyRouter, self).__init__(rule)
        self._root = root
        self._http = HttpClient(decompress=False, store_cookies=False)

    def get(self, request):
        # Combine our destination's root URL with this requests' relative path.
        url = self._root + request.path
        # Make a request with the asynchronous HTTP client.
        response = yield self._http.get(url)
        # Map the HttpResponse to the expected WsgiResponse.
        request.response.content = response.recv_body()
        request.response.content_type = response.info()['Content-Type']
        request.response.status_code = response.status_code
        request.response.headers['Access-Control-Allow-Origin'] = '*'
        # Return the updated WsgiResponse using magic.
        coroutine_return(request.response)  # asynchronous voodoo for "yield"


class Site(wsgi.LazyWsgi):
    # The half of the app that serves HTTP.
    def setup(self, environ):
        # I dunno this was cargo-culted from examples.
        cfg = environ['pulsar.cfg']
        loop = environ['pulsar.connection']._loop

        # Connect to the Redis-like state store shared between apps.
        self.store = create_store(cfg.data_store, loop=loop)

        # Create Handlers for three WebSockets and their data_store channels.
        return wsgi.WsgiHandler([  # route order is significant!
            ws.WebSocket('/scene', DirectorWS(self.store, 'scene')),
            ws.WebSocket(
                '/presentation', DirectorWS(self.store, 'presentation')),
            ws.WebSocket('/group', DirectorWS(self.store, 'group')),
            ProxyRouter('/director_api/<path:path>', DIRECTOR_API_URL),
            wsgi.MediaRouter('/', ASSET_DIR),  # static files
        ])


class Director(Application):
    """\
    A Pulsar Application bridging our state store and ROS topics.
    """

    def monitor_start(self, monitor):
        # We should only need one Worker.
        self.cfg.set('workers', 1)

    def publish_ros(self, channel, message):
        """\
        Publish the new state as a message on a ROS topic.
        """
        # This method is called by add_client() below.
        # http://pythonhosted.org/pulsar/apps/data/clients.html#pulsar.apps.data.store.PubSub.add_client

        # Use Interactive Spaces' own ROS message type.
        msg = GenericMessage()
        msg.type = 'json'
        msg.message = message  # verbatim

        if not rospy.is_shutdown():
            rospy.loginfo(msg)  # debug
            # Select the ROS topic for this channel and publish.
            self.pubs[channel].publish(msg)

    @task
    def fetch_next_scene(self):
        """\
        Begin an HTTP Request for the current Presentation's next Scene.
        """

        current_presentation = yield self.client.get('presentation')
        current_scene = yield self.client.get('scene')

        resource_uri = next_scene_uri(
            presentation=current_presentation,
            scene=current_scene
        )

        if resource_uri:
            url = DIRECTOR_API_URL + resource_uri
            self.logger.info("fetching " + url)
            # Begin HTTP Request, add callback for when it completes.
            self._http.get(url, post_request=self.new_scene)

    def new_scene(self, response, exc=None):
        """\
        Process an HTTP Response containing a new Scene.
        """
        scene = response.recv_body()
        self.pubsub.publish('scene', scene)
        self.client.set('scene', scene)

    def new_presentation(self, response, exc=None):
        """\
        Process an HTTP Response containing a new Presentation.
        """
        presentation = response.recv_body()
        self.pubsub.publish('presentation', presentation)
        self.client.set('presentation', presentation)

    def reset_scene_timer(self, worker, duration=3):
        """\
        Reset the Scene Timer countdown.
        """
        if self.scene_timer:
            self.scene_timer.cancel()
        self.scene_timer = worker._loop.call_later(
            duration, self.fetch_next_scene)
        rospy.loginfo('scene timer set for {0} seconds.'.format(duration))

    def worker_start(self, worker, exc=None):
        """\
        Called when the single Worker process begins.
        Setup connections to both the state datastore and ROS topics.
        """
        # Pulsar's Asynchronous HTTP Client
        self._http = HttpClient()

        # Provide access to this worker's logger.
        self.logger = worker.logger

        # Startup the ROS node.
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
        rospy.init_node('director')

        # Begin health checking.
        begin_checking_registration()

        # Instantiate three ROS topic Publishers, one for each store channel.
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Complete_example
        self.pubs = {
            'scene': rospy.Publisher(
                '/director/scene', GenericMessage, queue_size=10, latch=True),
            'presentation': rospy.Publisher(
                '/director/presentation', GenericMessage, queue_size=10, latch=True),
            'group': rospy.Publisher(
                '/director/group', GenericMessage, queue_size=10, latch=True),
        }

        # Connect to our internal Redis-like state store.
        # http://pythonhosted.org/pulsar/apps/data/clients.html
        self.store = create_store(self.cfg.data_store)
        # Create a client for the key/value store.
        self.client = self.store.client()
        # Create a client for the publish/subscribe service.
        self.pubsub = self.store.pubsub()
        # Subscribe to three channels.
        self.pubsub.subscribe('scene', 'presentation', 'group')
        # Add a callback for any new messages on these channels.
        self.pubsub.add_client(self.publish_ros)
        self.pubsub.add_client(Resetter(worker, self.reset_scene_timer))

        # Hold the Handle on the Scene Timer.
        self.scene_timer = None

    def worker_stopping(self, worker, exc=None):
        # Cleanly shutdown rospy node.  Probably not necessary.
        rospy.signal_shutdown("bye from %s" % worker.name)


# http://pythonhosted.org/pulsar/api/application.html#multi-app
class Server(MultiApp):
    ''' Create two pulsar applications, one for serving the web site
        and one for ROS communication.
    '''

    # Use Pulsar's data store, similar to Redis.
    # Share the same store URL between the apps.
    # http://pythonhosted.org/pulsar/apps/ds.html
    sys.argv = sys.argv[0:1]

    cfg = {
        'data_store': pulsards_url(),
        'bind': '0.0.0.0:8060'
    }

    def build(self):
        yield self.new_app(Director)
        yield self.new_app(wsgi.WSGIServer, callable=Site())


if __name__ == '__main__':
    try:
        Server('director').start()
    except rospy.ROSInterruptException:  # may not be needed
        pass
