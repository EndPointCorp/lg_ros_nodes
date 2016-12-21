#!/usr/bin/env python

import rospy
from lg_stats import main
from lg_common.helpers import write_influx_point_to_telegraf

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        data="""ros_respawns ros_node_name="%s",reason="%s",value=1" """ % ('lg_stats', e)
        write_influx_point_to_telegraf(data=data)
        rospy.sleep(1)
        raise
