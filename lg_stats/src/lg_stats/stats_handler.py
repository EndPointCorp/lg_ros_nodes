import json
import time
import rospy
import datetime
import traceback

from lg_common.helpers import write_influx_point_to_telegraf
from lg_common.logger import get_logger
logger = get_logger('stats_handler')


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
        logger.debug(f"handling stats for director: {scene}")
        # TODO remove this check for a bug
        if scene.get('data', None) is not None:
            scene = scene['data']
        if scene.get('slug', '') == 'attract-loop-break':
            # ignore this, if it's from the attract loop it
            # will publish activity=false and we'll handle
            # it there.
            logger.debug('ignoring attract-loop-break')
            return
        if self.last_presentation_start_time is not None:
            self.write_data()
        pres = {}
        pres['scene_name'] = scene.get('name', 'unknown')
        pres['slug'] = scene.get('slug', 'unknown')
        pres['presentation_name'] = scene.get('presentation', 'unknown')
        pres['type'] = scene.get('played_from', 'unknown')
        pres['created_by'] = scene.get('created_by', 'unknown')
        self.last_presentation_start_time = time.time()
        self.last_presentation = pres

    def handle_activity(self, msg):
        if self.active_state == msg.data:
            logger.debug('ignoring duplicate active_state')
            return  # nothing to do here
        self.active_state = msg.data
        if self.active_state is False:
            self.active_state = False
            self.write_data()

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
        if self.last_presentation.get('slug', '') != 'attract-loop-break':
            # only write the presentation when we're not in the attract loop
            pres = self.last_presentation
            logger.debug(f"touch_stats presentation_name=\"{pres['scene_name']}\",scene_name=\"{pres['scene_name']}\",type=\"{pres['type']}\",duration={duration},time_started={self.last_presentation_start_time}")
            write_influx_point_to_telegraf(f"touch_stats presentation_name=\"{pres['scene_name']}\",scene_name=\"{pres['scene_name']}\",type=\"{pres['type']}\",duration={duration},time_started={self.last_presentation_start_time}")
        else:
            logger.debug("ignored writing attract loop blank scene")

        # reset data to None
        self.last_presentation_start_time = None
        self.last_presentation = {}
