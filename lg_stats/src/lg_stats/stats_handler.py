import json
import time
import rospy
import datetime
import traceback

from lg_common.helpers import write_influx_point_to_telegraf, get_hostname
from lg_common.logger import get_logger
logger = get_logger('stats_handler')


def escape_tag_value(value):
    """
    Escapes commas, equal signs, and spaces in tag values with a single backslash.
    """
    return value.replace(" ", "\\ ").replace(",", "\\,").replace("=", "\\=")

def escape_field_value(value):
    """
    Escapes double quotes and backslashes in field values with a single backslash.
    """
    return value.replace('"', '\\"').replace("\\", "\\")



class StatsHandler():
    def __init__(self):
        # set up stats writer here
        self.last_presentation_start_time = None
        self.last_presentation = {}
        self.active_state = False

    def handle_director(self, msg):
        try:
            self._handle_director(msg)
        except Exception as e:
            logger.error(f"exception was {e}")
            traceback.print_exc()

    def _handle_director(self, msg):
        scene = json.loads(msg.message)
        logger.debug(f"handling stats for director")
        if scene.get('data', None) is not None:
            scene = scene['data']
        if scene.get('played_from', '') == 'lg_attract_loop':
            # ignore this, if it's from the attract loop it
            # will publish activity=false and we'll handle
            # it there.
            logger.debug('ignoring attract-loop-break')
            return
        if scene.get('slug', '') == 'stop-the-presentations':
            # ignore this as well
            logger.debug('ignoring stop-the-presentations messages')
            return
        if self.last_presentation_start_time is not None and self.active_state:
            # System is active and we have a new presentation starting,
            # then in that case close the stats for the previous presentation and push the data point
            self.write_data()
        pres = {}
        pres['scene_name'] = scene.get('name', 'unknown')
        pres['slug'] = scene.get('slug', 'unknown')
        pres['presentation_name'] = scene.get('name', 'unknown')
        pres['presentation_id'] = scene.get('presentation_id', 'unknown')
        pres['type'] = scene.get('played_from', 'unknown')
        pres['created_by'] = scene.get('created_by', 'unknown')
        pres['hostname'] = get_hostname()
        pres['played_from'] = scene.get('played_from', '')
        if self.active_state:
            # if the system is active, then start the timer to track the active duration
            self.last_presentation_start_time = time.time()
            logger.debug(f"System is active, so starting the time counter/")
        self.last_presentation = pres
        logger.debug(f"Director msg stored as last_presentation: {self.last_presentation}")

    def handle_activity(self, msg):
        logger.debug(f"handling stats for activity: {msg}")
        if self.active_state == msg.data:
            logger.debug('ignoring duplicate active_state')
            return  # nothing to do here
        self.active_state = msg.data

        if self.active_state is False:
            logger.debug('activity is False, writing data')
            self.write_data()
            self.last_presentation_start_time = None

        if self.active_state is True:
            if self.last_presentation:
                logger.debug(f'activity is True, setting presentation start time, starting timer. Presentation is {self.last_presentation}')
                self.last_presentation_start_time = time.time()

    def write_data(self):
        try:
            self._write_data()
        except Exception as e:
            logger.error(e)
            traceback.print_exc()

    def _write_data(self):
        # write data here
        if self.last_presentation_start_time is None:
            return
        duration = time.time() - self.last_presentation_start_time
        # TODO also check for ['source'] to make sure it is from a valid source
        if self.last_presentation.get('played_from', '') != 'lg_attract_loop':
            # only write the presentation when we're not in the attract loop
            pres = self.last_presentation
            time_started = datetime.datetime.fromtimestamp(self.last_presentation_start_time)

            tag_presentation_name = escape_tag_value(pres['presentation_name'])
            tag_presentation_id = escape_tag_value(pres['presentation_id'])
            tag_played_from = escape_tag_value(pres['played_from'])
            field_time_started = escape_field_value(str(time_started))
            field_scene_name = escape_field_value(pres['scene_name'])
            field_type = escape_field_value(pres['type'])

            query = f"touch_stats,presentation_name={pres['presentation_name']},played_from={pres['played_from']},presentation_id={pres['presentation_id']} scene_name={pres['scene_name']},type={pres['type']},duration={duration},time_started={time_started}"
            logger.debug(f"Writing the data point to influxdb: {query}")
            write_influx_point_to_telegraf(query)
        else:
            logger.debug("ignored writing attract loop messages")

        self.last_presentation_start_time = None

