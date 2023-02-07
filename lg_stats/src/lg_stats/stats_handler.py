import json
import time
import datetime


class StatsHandler():
    def __init__(self):
        # set up stats writer here
        self.last_presentation_start_time = None
        self.last_presentation = {}
        self.active_state = False

    def handle_director(self, msg):
        scene = json.loads(msg.message)
        print(f"got a new scene name: '{scene['name']}'")
        if self.last_presentation_start_time is not None:
            self.write_data()
        self.last_presentation_start_time = time.time()
        pres = {}
        pres['name'] = scene['name']
        pres['slug'] = scene['slug']
        self.last_presentation = pres

    def handle_activity(self, msg):
        print(f'got activity: {msg.data}')
        self.active_state = msg.data
        if self.active_state == msg.data:
            return  # nothing to do here
        if self.active_state is False:
            self.active_state = False
            self.write_data()

    def write_data(self):
        print('going to write scene data')
        # write data here
        duration = time.time() - self.last_presentation_start_time
        # TODO also check for ['source'] to make sure it is from a valid source
        if self.last_presentation['slug'] != 'attract-loop-break':
            # only write the presentation when we're not in the attract loop
            print(f"duration: {duration}\nname: {self.last_presentation['name']}\ntime started: {datetime.datetime.fromtimestamp(self.last_presentation_start_time)}")
        else:
            print("DEBUG: ignored writing attract loop blank scene")

        # reset data to None
        self.last_presentation_start_time = None
        self.last_presentation = {}
