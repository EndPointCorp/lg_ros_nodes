import subprocess
import rospy
import rosservice

class KmlAlive:
    def __init__(self, earth_proc):
        self.earth_proc = earth_proc
        rospy.sleep(5)
        self.keep_alive()

    def keep_alive(self):
        loop_timeout = 0.2
        with open('/dev/null', 'w') as dev_null:
            while not rospy.is_shutdown():
                try:
                    pid = self.earth_proc.proc.watcher.proc.pid
                except AttributeError:
                    rospy.sleep(loop_timeout)
                if '/kmlsync/state' in rosservice.get_service_list():
                    cmd = "lsof -Pn -p {} -a -i @127.0.0.1:8765".format(pid).split(' ')
                        ret_value = subprocess.call(
                            cmd,
                            stdout=dev_null,
                            stderr=dev_null,
                            close_fds=True
                        )
                        if ret_value != 0:
                            self.earth_proc.handle_soft_relaunch()
               rospy.sleep(loop_timeout)
