import subprocess
import rospy
import traceback
import sys
from lg_common.logger import get_logger
logger = get_logger('kmlalive')


class KmlAlive:
    def __init__(self, earth_proc):
        self.earth_proc = earth_proc
        logger.debug("starting KMLALIVE process")
        self.timeout_period = rospy.get_param('~timeout_period', 5)
        self.initial_timeout = rospy.get_param('~initial_timeout', 60)
        rospy.Timer(rospy.Duration(10), self.keep_alive, oneshot=True)
        # only restart when worked is true, otherwise
        # it may have never worked
        self.worked = False

    def keep_alive(self, *args, **kwargs):
        logger.debug("just in first keep_alive")
        loop_timeout = 1
        counter = 0
        rospy.sleep(1)
        while not rospy.is_shutdown():
            try:
                pid = self.earth_proc.proc.watcher.proc.pid
            except AttributeError as e:
                counter = 0
                logger.warning("Earth proc doesn't exist {}".format(e))
                rospy.sleep(loop_timeout)
                continue
            cmd = "lsof -Pn -p {} -a -i @localohst:8765".format(pid).split(' ')
            ret_value = subprocess.call(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True
            )
            if ret_value == 0:
                self.worked = True
                counter = 0
            else:
                counter += 1
                logger.info("found non zero value for {} counter at {}".format(pid, counter))
                if (counter > self.timeout_period and self.worked) or counter > self.initial_timeout:
                    logger.error("RELAUNCHING worked: {}  counter: {}".format(self.worked, counter))
                    self.earth_proc.handle_soft_relaunch()
                    counter = 0
                    self.worked = False
            rospy.sleep(loop_timeout)
