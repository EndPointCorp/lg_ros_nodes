import subprocess
import rospy
import rosservice


class KmlAlive:
    def __init__(self, earth_proc):
        self.earth_proc = earth_proc
        rospy.loginfo("XXX starting KMLALIVE process")
        rospy.Timer(rospy.Duration(10), self.keep_alive, oneshot=True)

    def keep_alive(self, *args, **kwargs):
        try:
            self._keep_alive(args, kwargs)
        except Exception as e:
            rospy.logerr("exception was {}".format(e))
            rospy.sleep(1)
            self.keep_alive(args, kwargs)

    def _keep_alive(self, *args, **kwargs):
        rospy.logerr("XXX in first keep_alive")
        loop_timeout = 1
        counter = 0
        with open('/dev/null', 'w') as dev_null:
            while not rospy.is_shutdown():
                try:
                    pid = self.earth_proc.proc.watcher.proc.pid
                except AttributeError as e:
                    counter = 0
                    rospy.logwarn("Earth proc doesn't exist {}".format(e))
                    rospy.sleep(loop_timeout)
                    continue
                if '/kmlsync/state' in rosservice.get_service_list():
                    cmd = "lsof -Pn -p {} -a -i @127.0.0.1:8765".format(pid).split(' ')
                    ret_value = subprocess.call(
                        cmd,
                        stdout=dev_null,
                        stderr=dev_null,
                        close_fds=True
                    )
                    if ret_value != 0:
                        counter += 1
                        rospy.logerr("XXX found non zero value for {} counter at {}".format(pid, counter))
                        if counter > 10:
                            rospy.logerr("XXX RELAUNCHING")
                            self.earth_proc.handle_soft_relaunch()
                            counter = 0
                else:
                    rospy.logerr("no kml sync state found")
                rospy.sleep(loop_timeout)
