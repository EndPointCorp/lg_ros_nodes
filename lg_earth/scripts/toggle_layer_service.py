#!/usr/bin/env python

import rospy

from lg_earth import Toggle3d
from lg_earth.srv import Toggle3dState, Toggle3dSet

from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'earth_3d_layer_service'

def main():
    rospy.init_node(NODE_NAME, anonymous=False)

    s = Toggle3d()
    def set_service_handler(msg):
        s.set_layer_state(msg.state)
    
    def state_service_handler(msg):
        return s.get_state

    rospy.Service('/earth/toggle_3d_layer_set', Toggle3dSet, set_service_handler)
    rospy.Service('/earth/toggle_3d_layer_state', Toggle3dState, state_service_handler)

    rospy.spin()

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)