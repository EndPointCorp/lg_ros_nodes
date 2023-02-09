import json
import time
import datetime
from lg_common.helpers import write_influx_point_to_telegraf


class StatsHandler():
    def __init__(self):
        # set up stats writer here
        self.last_presentation_start_time = None
        self.last_presentation = {}
        self.active_state = False

    def handle_director(self, msg):
        scene = json.loads(msg.message)
        if scene['slug'] == 'attract-loop-break':
            # ignore this, if it's from the attract loop it
            # will publish activity=false and we'll handle
            # it there.
            return
        if self.last_presentation_start_time is not None:
            self.write_data()
        self.last_presentation_start_time = time.time()
        pres = {}
        pres['scene_name'] = scene['name']
        pres['slug'] = scene['slug']
        pres['presentation_name'] = scene.get('presentation', 'unknown')
        pres['type'] = scene.get('played_from', 'unknown')
        pres['created_by'] = scene.get('created_by', 'unknown')
        self.last_presentation = pres

    def handle_activity(self, msg):
        if self.active_state == msg.data:
            return  # nothing to do here
        self.active_state = msg.data
        if self.active_state is False:
            self.active_state = False
            self.write_data()

    def write_data(self):
        # write data here
        duration = time.time() - self.last_presentation_start_time
        # TODO also check for ['source'] to make sure it is from a valid source
        if self.last_presentation['slug'] != 'attract-loop-break':
            # only write the presentation when we're not in the attract loop
            pres = self.last_presentation
            print(f"touch_stats presentation_name='{pres['scene_name']}',scene_name='{pres['scene_name']}',type='{pres['type']}',duration='{duration}',time_started='{self.last_presentation_start_time}'")
            write_influx_point_to_telegraf(f"touch_stats presentation_name='{pres['scene_name']}',scene_name='{pres['scene_name']}',type='{pres['type']}',duration='{duration}',time_started='{self.last_presentation_start_time}'")
        else:
            print("DEBUG: ignored writing attract loop blank scene")

        # reset data to None
        self.last_presentation_start_time = None
        self.last_presentation = {}
