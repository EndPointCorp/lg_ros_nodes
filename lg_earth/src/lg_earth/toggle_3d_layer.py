import subprocess

class Toggle3d:
    def __init__(self):
        self.layer_is_on = True
    
    def set_layer_state(self, state):
        self.sync_state_to_earth(state)
        self.layer_is_on = state  

    def get_state(self):
        return self.layer_is_on

    def sync_state_to_earth(self, state):
        subprocess.call(['toggle_layer.bash', 'on' if state else 'off'])