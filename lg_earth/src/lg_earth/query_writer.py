import rospy
from .query_queue import QueryQueue
from lg_common.logger import get_logger
logger = get_logger('query_writer')
ABSOLUTE_VIEW_QUERY = 'absolute_view'


class QueryWriter:
    def __init__(self, filename, maxlen=10):
        self.filename = filename
        self._queue = QueryQueue(self.filename, maxlen=maxlen)

    def post_query(self, query, coalesce_key=None):
        logger.debug('posting query: {}'.format(query))
        self._queue.post_query(query, coalesce_key=coalesce_key)

    def shutdown(self):
        self._queue.stop()

    def handle_flyto_kml(self, msg):
        query = 'flytoview={}'.format(msg.data)
        self.post_query(query, coalesce_key=ABSOLUTE_VIEW_QUERY)

    def handle_flyto_pose_camera(self, pose):
        logger.debug(pose)
        kml = ('<Camera><latitude>{}</latitude>'
               '<longitude>{}</longitude><altitude>{}</altitude>'
               '<heading>{}</heading><tilt>{}</tilt><roll>{}</roll>'
               '<altitudeMode>absolute</altitudeMode></Camera>').format(
            pose.position.y,
            pose.position.x,
            pose.position.z,
            pose.orientation.z,
            pose.orientation.x,
            pose.orientation.y)
        self.post_query(
            'flytoview={}'.format(kml),
            coalesce_key=ABSOLUTE_VIEW_QUERY,
        )

    def handle_flyto_pose_lookat(self, pose):
        logger.debug(pose)
        kml = ('<LookAt><latitude>{}</latitude>'
               '<longitude>{}</longitude><altitude>{}</altitude>'
               '<heading>{}</heading><tilt>{}</tilt><range>{}</range>'
               '<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>').format(
            pose.position.y,
            pose.position.x,
            pose.position.z,
            pose.orientation.z,
            pose.orientation.x,
            pose.orientation.y)
        self.post_query('flytoview={}'.format(kml))

    def handle_search(self, msg):
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
