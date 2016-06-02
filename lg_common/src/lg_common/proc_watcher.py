import os
import rospy
import psutil
import subprocess
from time import sleep, time
from appctl.srv import NodeQuery
from lg_common.msg import ApplicationState
from lg_common.helpers import write_log_to_file

write_log_to_file = rospy.loginfo


MAX_OUT_OF_BOUNDS = 7

class ProcWatcher(object):
    def __init__(self, node_name, cpu_time=None, inactive_time=30,
                 cpu_usage=None, memory_usage=None, diskspace_usage=None,
                 diskspace_dirs=None):
        if diskspace_dirs is None:
            write_log_to_file('no diskspace dirs...')
            diskspace_dirs = []
        # limits for usage
        self.cpu_time = cpu_time
        self.inactive_time = inactive_time
        # cpu percent
        self.cpu_usage = cpu_usage
        self.cpu_usage_history = []
        # max megabytes (can change to percent if desired)
        self.memory_usage = memory_usage
        self.memory_usage_history = []
        # max gigabytes
        self.diskspace_usage = diskspace_usage
        # the dirs to check
        if type(diskspace_dirs) == list:
            self.diskspace_dirs = diskspace_dirs
        else:
            self.diskspace_dirs = []
        # the proc runner
        self.proc = None
        # the pid from that proc runner
        self.pid = None
        # the psutil object
        self.ps = None
        # tracking of the app state TODO start using this
        self.app_state = ApplicationState.VISIBLE
        # time the state was changed
        self.app_state_changed = time()
        # if badstate is true, reset after !is_visible for inactive_timeout
        # seconds
        self.badstate = {}

        # communication with the process is done here
        self.appctl_service = rospy.ServiceProxy(
            '/appctl' + node_name + '/query', NodeQuery)
        write_log_to_file('please at least this should work...')

    def set_state(self, state):
        self.app_state = state
        self.app_state_changed = time()

    def get_pid(self):
        return int(self.appctl_service('get_pid').resp)

    def run(self, *args, **kwargs):
        """
        Needs to do stuff in the background
        """
        pid = self.get_pid()
        # if this is a new process, we should clear the state
        if pid != self.pid:
            write_log_to_file('proc has changed from %s' % self.pid)
            self._proc_has_reset()
            self.pid = pid
        if self.pid == 0:
            # 0 signifies error
            write_log_to_file('proc id cannot be found...')
            return
        try:
            self.ps = psutil.Process(self.pid)
            if self.cpu_time:
                self._check_cpu_time()
            if self.cpu_usage:
                self._check_cpu_usage()
            if self.memory_usage:
                self._check_memory_usage()
            if self.diskspace_usage and self.diskspace_dirs:
                self._check_diskspace_usage()
            if self.badstate:
                self._handle_bad_state()
        except psutil.NoSuchProcess:
            # the process no longer exists
            if self.badstate:
                self._handle_bad_state()
            return

    def is_visible(self):
        return self.app_state == ApplicationState.VISIBLE

    def _check_cpu_time(self):
        """
        Checks cpu time, if > self.cpu_time set badstate := True
        """
        write_log_to_file('about to check...')
        times = self.ps.cpu_times()
        write_log_to_file('found cpu time of %s' % times.user + times.system)
        if (times.user + times.system) > self.cpu_time:
            self.badstate = True

    def _check_cpu_usage(self):
        """
        Checks cpu percent usage, must remain consistent for N seconds
        because sometimes cpu usage spikes and we shouldn't worry about
        cpu usage unnecessarily
        """
        percent = self.ps.cpu_percent()
        self._append_and_check(self.cpu_usage_history, percent, self.cpu_usage)
        self.cpu_usage_history = self.cpu_usage_history[-10:]

    def _append_and_check(self, arr, new_value, maxx):
        """
        General check for adding a new value in an array, and finding
        if > MAX_OUT_OF_BOUNDS values are greater than maxx
        """
        arr.append(new_value)
        # count out previous cpu usages over self.cpu_usage
        n = len([p for p in arr if p > maxx])
        write_log_to_file('array looks like %s' % arr)
        if n > MAX_OUT_OF_BOUNDS:
            self.badstate = True

    def _check_memory_usage(self):
        """
        Checks memory percent usage, must remain consistent for N seconds
        because sometimes memory usage spikes and we shouldn't worry about
        memory usage unnecessarily
        """
        # grab rss from memory info (ignore virtual memory)
        # also convert from bytes to MB
        usage = self.ps.memory_info().rss / 1024**2
        write_log_to_file('found usage: %s' % usage)
        self._append_and_check(self.memory_usage_history, usage,
                               self.memory_usage)
        self.memory_usage_history = self.memory_usage_history[-10:]

    def _check_diskspace_usage(self):
        """
        Looks at the disk usage in the list of directories in
        self.diskspace_dirs and compare to self.diskspace_usage
        """
        usage = 0
        for d in self.diskspace_dirs:
            if not os.path.exists(d):
                continue
            p = subprocess.Popen(['du', '-s', d], stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            out = p.communicate()
            try:
                usage += int(out[0].split('\t')[0])
            except ValueError:
                continue

        write_log_to_file('usage is: %s' % float(usage /1024.0))
        if (usage / 1024.0) > self.diskspace_usage:
            self.badstate = True

    def _proc_has_reset(self):
        self.cpu_usage_history = []
        self.memory_usage_history = []
        self.badstate = False

    def _handle_bad_state(self):
        """
        Needs to be in a !visible state, and have been set that way at
        least self.inactive_time seconds ago
        """
        write_log_to_file('handling a bad state...')
        if not self.is_visible() and  \
                (time() - self.app_state_changed) > self.inactive_time:
            self.ps.kill()
