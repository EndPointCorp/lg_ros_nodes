#!/usr/bin/env python

# This is heavily influenced by Pulsar's examples,
# especially `chat`, `tweets`, and `philosophers`.
# http://pythonhosted.org/pulsar/tutorials/index.html

import os
import sys
import json
import rospy
import signal
import requests


from pulsar.apps import wsgi, ws, Application, MultiApp
from pulsar.apps.data import create_store
from pulsar.apps.ds import pulsards_url
from pulsar.apps.http import HttpClient
from pulsar import command, task, coroutine_return
from std_msgs.msg import Bool

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
            return # nevermind

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

    def on_open(self, websocket): # When a new connection is established
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
        coroutine_return(request.response) # asynchronous voodoo for "yield"


class Site(wsgi.LazyWsgi):
    # The half of the app that serves HTTP.
    def setup(self, environ):
        # I dunno this was cargo-culted from examples.
        cfg = environ['pulsar.cfg']
        loop = environ['pulsar.connection']._loop

        # Connect to the Redis-like state store shared between apps.
        self.store = create_store(cfg.data_store, loop=loop)

        # Create Handlers for three WebSockets and their data_store channels.
        return wsgi.WsgiHandler([ # route order is significant!
            ws.WebSocket('/scene', DirectorWS(self.store, 'scene')),
            ws.WebSocket(
                '/presentation', DirectorWS(self.store, 'presentation')),
            ws.WebSocket('/group', DirectorWS(self.store, 'group')),
            ProxyRouter('/director_api/<path:path>', DIRECTOR_API_URL),
            wsgi.MediaRouter('/', ASSET_DIR), # static files
        ])


class Director(Application):
    """\
    A Pulsar Application bridging our state store and ROS topics.
    """

    def monitor_start(self, monitor):
        # We should only need one Worker.
        self.cfg.set('workers', 1)

    def publish_ros(self, channel, message, msg_type=None):
        """\
        Publish the new state as a message on a ROS topic.
        """
        # This method is called by add_client() below.
        # http://pythonhosted.org/pulsar/apps/data/clients.html#pulsar.apps.data.store.PubSub.add_client

        # Use Interactive Spaces' own ROS message type.
        if msg_type == 'Bool':
            msg = Bool()
            msg.data = message
        else:
            msg = GenericMessage()
            msg.type = 'json'
            msg.message = message # verbatim

        if not rospy.is_shutdown():
            rospy.loginfo(msg) # debug
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
        self.activity_topic = rospy.get_param("~activity_topic", "/activity/active")
        # Pulsar's Asynchronous HTTP Client
        self._http = HttpClient()

        # Initialize attract loop queue. Attract loop contains dicts with {'scene': scene, 'presentation': presentation}
        self.attract_loop_queue = []

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
            'scene': rospy.Publisher('/director/scene',
            GenericMessage, queue_size=10, latch=True),
            'presentation': rospy.Publisher('/director/presentation',
            GenericMessage, queue_size=10, latch=True),
            'group': rospy.Publisher('/director/group',
            GenericMessage, queue_size=10, latch=True),
            'touchscreen': rospy.Publisher('/director/touchscreen',
            GenericMessage, queue_size=10, latch=True),
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

        #subscribe to activity topic to enable attract loop support
        rospy.Subscriber(self.activity_topic, Bool, self._process_activity_state_change)

        rospy.loginfo("Director subscribed to activity topic for attract loop: %s" % self.activity_topic)

        # Hold the Handle on the Scene Timer.
        self.scene_timer = None

    def _process_activity_state_change(self, message):
        """
        Method responsible for starting (or continuing) of playback for attract loop.
        """
        if message.data == True:
            rospy.loginfo("Director: Attract loop becoming inactive")
            self._stop_attract_loop()
        elif message.data == False:
            rospy.loginfo("Director: Attract loop becoming active")
            if len(self.attract_loop_queue) > 0:
                rospy.loginfo("Continuing attract loop")
                self._play_attract_loop(cont=True)
            else:
                rospy.loginfo("Starting attract loop")
                self._play_attract_loop()
        else:
            rospy.logerr("Activity message contained unknown state")

    def _stop_attract_loop(self):
        rospy.loginfo("Stopping scene timer")
        #self.scene_timer.cancel()
        return

    def _play_attract_loop(self, cont=False):
        """
        Check if there are scenes to continue the attract loop or fetch them and play them back
        """
        if not cont:
            rospy.loginfo("Fetching scenes from presentations marked as attract loop")
            #/try:
            self.attract_loop_queue = self._fetch_attract_loop_content()['scenes']
            rospy.loginfo("Populated attract_loop_queue with %s" % self.attract_loop_queue)
            self._play_attract_loop_content()
            #except Exception, e:
            #    rospy.logerr("Failed to populate attract loop queue with content because %s" % e)
        else:
            self._play_attract_loop_content()

    def _play_attract_loop_content(self):
        rospy.loginfo("Executing _play_attract_loop_content")
        opts = '?format=json'
        #while self.attract_loop_queue:
        scene_presentation = self.attract_loop_queue.pop(0)
        scene = scene_presentation['scene'] # bare object with resource_uri
        scene_url = "%s%s%s" % (DIRECTOR_API_URL, scene['resource_uri'], opts)
        presentation = scene_presentation['presentation'] # bare object with resource_uri
        presentation_url = "%s%s%s" % (DIRECTOR_API_URL, presentation['resource_uri'], opts)
        full_scene = json.dumps(json.loads(requests.get(scene_url).content)) # ROS nodes understandable full scene
        rospy.loginfo("Playing attract loop URIs. Presentation: %s, scene: %s" % (presentation_url, scene_url))

        presentation_response = yield self._http.get(presentation_url, post_request=self.new_presentation)
        scene_response = yield self._http.get(scene_url, post_request=self.new_scene)

        self.new_scene(scene_response)
        self.new_presentation(presentation_response)

    def _fetch_attract_loop_content(self):
        """
        Fetch presentation groups, presentations and scenes for attract loop.
        Return a dict with the content wherE:
        - presentationgroups is a list of presentationgroups
        - presentations is a list of presentations
        - scenes is alist of dictionaries containing one scene and presentation that it belongs to
            {'scene': <scene>, 'presentation': <presentation>}
        """
        client = requests
        presentationgroups = self._fetch_attract_loop_presentationgroups(client)
        if presentationgroups:
            presentations = self._fetch_presentationgroup_presentations(client, presentationgroups)
            scenes = self._fetch_scenes_from_presentations(client, presentations)
            content =  {'presentationgroups': presentationgroups,
                        'presentations': presentations,
                        'scenes': scenes }
            #rospy.loginfo("Returning content for attract loop: %s" % content)
            return content
        else:
            rospy.loginfo("No presentation groups found in attract loop")
            return

    def _fetch_scenes_from_presentations(self, client, presentations):
        """
        Get all scenes from presentations and fetch the object directly
        so the list is ready to be publised on /director/scene

        Returned list should contain dictionaries like {'scene': <scene>, 'presentation': <presentation>}
        """
        fetched_scenes = []
        for presentation in presentations:
            try:
                presentation_resource_uri = presentation['resource_uri']
                presentation_request = client.get("%s%s" % (DIRECTOR_API_URL, presentation_resource_uri)).content
                presentation_scenes = json.loads(presentation_request)['scenes']
            except Exception, e:
                rospy.logerr("Could not fetch presentation scenes from presentations (%s) because %s" % (presentations, e))
                return []

            for scene in presentation_scenes:
                try:
                    #scene_resource_uri = scene['resource_uri']
                    #scene_request = client.get("%s%s" % (DIRECTOR_API_URL, scene_resource_uri)).content
                    #scene = json.loads(scene_request)
                    pass
                except Exception, e:
                    rospy.logerr("Could not fetch scene (%s) from presentation scenes (%s) because %s" % (scene, presentation_scenes, e))
                    return []
                fetched_scenes.extend([{'presentation': presentation, 'scene': scene}])

        #rospy.loginfo("Fetched scenes: %s" % fetched_scenes)
        return fetched_scenes

    def _fetch_presentationgroup_presentations(self, client, presentationgroups):
        """
        """
        attract_loop_presentations = []
        try:
            for presentation in presentationgroups:
                presentation_resource_uri = presentation['resource_uri']
                presentations_request = client.get("%s%s" % (DIRECTOR_API_URL, presentation_resource_uri)).content
                presentations = json.loads(presentations_request)['presentations']
                attract_loop_presentations.extend(presentations)
            return attract_loop_presentations
        except Exception, e:
            rospy.logerr("Could not fetch presentations from presentationgroups (%s) because %s" % (presentationgroups, e))
            return []

    def _fetch_attract_loop_presentationgroups(self, client):
        try:
            presentationgroup_request = client.get("%s/director_api/presentationgroup/?attract_loop=True" % DIRECTOR_API_URL).content
            presentationgroups = json.loads(presentationgroup_request)['objects']
            assert(type(presentationgroups) == list), "Presentationgroups type is not list"
            return presentationgroups
        except Exception, e:
            rospy.logerr("Could not get presentationgroups because: %s" % e)
            return []

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
    except rospy.ROSInterruptException: # may not be needed
        pass
