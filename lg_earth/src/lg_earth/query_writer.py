import rospy
from std_msgs.msg import String
from .query_queue import QueryQueue
from lg_common.logger import get_logger
logger = get_logger('query_writer')


class QueryWriter:
    def __init__(self, filename, maxlen=10):
        self.filename = filename
        self._queue = QueryQueue(self.filename, maxlen=maxlen)

    def post_query(self, query):
        logger.debug('posting query: {}'.format(query))
        self._queue.post_query(query)

    def shutdown(self):
        self._queue.stop()

    def handle_flyto_kml(self, msg):
        kml = msg.data
        query = 'flytoview={}'.format(kml)
        self.post_query(query)

    def handle_flyto_pose_camera(self, msg):
        pose = msg
        logger.debug(msg)
        pose_lon = pose.position.x
        pose_lat = pose.position.y
        pose_alt = pose.position.z
        pose_heading = pose.orientation.z
        pose_tilt = pose.orientation.x
        pose_roll = pose.orientation.y
        kml = ('<Camera><latitude>{}</latitude>'
               '<longitude>{}</longitude><altitude>{}</altitude>'
               '<heading>{}</heading><tilt>{}</tilt><roll>{}</roll>'
               '<altitudeMode>absolute</altitudeMode></Camera>').format(
            pose_lat,
            pose_lon,
            pose_alt,
            pose_heading,
            pose_tilt,
            pose_roll)

        query = 'flytoview={}'.format(kml)
        self.post_query(query)

    def handle_flyto_pose_lookat(self, msg):
        pose = msg
        logger.debug(msg)
        pose_lon = pose.position.x
        pose_lat = pose.position.y
        pose_alt = pose.position.z
        pose_heading = pose.orientation.z
        pose_tilt = pose.orientation.x
        pose_range = pose.orientation.y
        kml = ('<LookAt><latitude>{}</latitude>'
               '<longitude>{}</longitude><altitude>{}</altitude>'
               '<heading>{}</heading><tilt>{}</tilt><range>{}</range>'
               '<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>').format(
            pose_lat,
            pose_lon,
            pose_alt,
            pose_heading,
            pose_tilt,
            pose_range)

        query = 'flytoview={}'.format(kml)
        self.post_query(query)

    def handle_search(self, msg):
        # call handle_tour with empty tour to stop any flyto in progress
        #self.handle_tour(String())
        search_query = msg.data
        query = 'search={}'.format(search_query)
        self.post_query(query)

    def handle_planet(self, msg):
        planet_name = msg.data
        query = 'planet={}'.format(planet_name)
        self.post_query(query)

    def handle_tour(self, msg):
        tour_var = msg.data

        if not tour_var:
            query = 'exittour=true'
        else:
            query = 'playtour={}'.format(tour_var)
            logger.debug(tour_var)
        self.post_query(query)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
