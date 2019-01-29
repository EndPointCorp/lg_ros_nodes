import subprocess

class Toggle3d:
    def __init__(self):
        self.layer_is_on = True
    
    def set_layer_state(self, state):
        self.sync_state_to_earth(state)
        self.layer_is_on = state
        
        if self.state_listener:
            self.state_listener(self.layer_is_on)

    def set_on_change_listener(self, state_listener):
        self.state_listener = state_listener

    def sync_state_to_earth(self, state):
        subprocess.call(['/home/lg/bash_scripts/toggle_layer.bash', 'on' if state else 'off'])