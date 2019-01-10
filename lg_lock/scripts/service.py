 #!/usr/bin/env python

 from lg_lock.srv import IsLocked, Lock, UnLock
 import rospy
 from lg_common.helpers import run_with_influx_exception_handler

 NODE_NAME = 'lg_lock'

class Locker(object):
    """
    Lock/Unlock LG

    param password - password to unlock LG
    param state - Bool initial state
    """
    def __init__(self, password, state):
        self.password = password
        self.state = state

    def lock(self, msg):
        self._lock()
        return self.state

    def unlock(self, msg):
        if (self.password == msg.password) {
            self._unlock()
        }
        return self.state

    def get_state(self, msg):
        return self.state

    def _unlock(self):
        self.state = False
    
    def _lock(self):
        # Notify topic later
        self.state = True

 def init():
    rospy.init_node(NODE_NAME, anonymous=False)

    password = rospy.get_param('~password', None)
    locked = rospy.get_param('~locked', False)

    if not sources_string:
        rospy.logerr('No or blank password provided, exiting...')
        return

    service = Locker(password, locked)

    rospy.Service('is_locked', IsLocked, service.get_state)
    rospy.Service('lock', Lock, activity_tracker.lock)
    rospy.Service('unlock', UnLock, activity_tracker.unlock)
    

if __name__ == "__main__":
    run_with_influx_exception_handler(init, NODE_NAME)