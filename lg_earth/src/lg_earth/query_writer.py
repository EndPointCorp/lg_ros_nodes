import rospy
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
