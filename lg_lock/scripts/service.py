 #!/usr/bin/env python

 from lg_lock.srv import IsLocked, Lock, UnLock
 from lg_lock.msg import State
 from lg_lock import LockerService
 import rospy
 from lg_common.helpers import run_with_influx_exception_handler

 NODE_NAME = 'lg_lock'

 def init():
    rospy.init_node(NODE_NAME, anonymous=False)

    password = rospy.get_param('~password', None)
    locked = rospy.get_param('~locked', False)
    
    statePublisher = rospy.Publisher('/lg_lock/locked', State, queue_size=1, latch=True)

    if not sources_string:
        rospy.logerr('No or blank password provided, exiting...')
        return

    service = LockerService(statePublisher, password, locked)

    rospy.Service('is_locked', IsLocked, service.get_state)
    rospy.Service('lock', Lock, activity_tracker.lock)
    rospy.Service('unlock', UnLock, activity_tracker.unlock)
    

if __name__ == "__main__":
    run_with_influx_exception_handler(init, NODE_NAME)