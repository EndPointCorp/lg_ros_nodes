import os
import signal
import threading

import rospy
from appctl_support import ProcController
from lg_common.msg import ProcessState


class ManagedApplication(object):
    def __init__(self, cmd, window=None, suspend=False):
        self.cmd = cmd
        self.window = window
        self.suspend = suspend
        self.state = ProcessState.STOPPED

        self.lock = threading.Lock()
        self.proc = ProcController(cmd)
        self.sig_timer = None

        rospy.on_shutdown(self._cleanup)

    # explicit SIGCONT needed to prevent undeath
    def _cleanup(self):
        if self.suspend:
            self._signal_proc(signal.SIGCONT, retry=False)

    # TODO(mv): better pid retrieval and/or signalling as a feature of ProcController
    def _signal_proc(self, sig, retry=True):
        if self.sig_timer is not None:
            self.sig_timer.shutdown()
        if self.proc.watcher is not None and self.proc.watcher.proc is not None:
            pid = self.proc.watcher.proc.pid
            try:
                os.kill(pid, sig)
                print "Sent signal {} to pid {}".format(sig, pid)
            except OSError:
                print "OSError while sending signal {} to pid {}".format(sig, pid)
        else:
            print "Can't signal, no proc"
            if not retry:
                return

            def retry_signal(ev):
                self._signal_proc(sig)
            self.sig_timer = rospy.Timer(rospy.Duration(0.1), retry_signal, oneshot=True)

    def get_state(self):
        return self.state

    def set_state(self, state):
        state_changed = False
        with self.lock:
            if state != self.state:
                state_changed = True
            self.state = state

            if not state_changed:
                return

            if state == ProcessState.STOPPED:
                print "STOPPED"
                if self.suspend:
                    self._signal_proc(signal.SIGCONT)
                self.proc.stop()
                if self.window is not None:
                    self.window.set_visibility(False)

            elif state == ProcessState.RUNNING:
                print "RUNNING"
                self.proc.start()
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                if self.suspend:
                    self._signal_proc(signal.SIGSTOP)

            elif state == ProcessState.ACTIVE:
                print "ACTIVE"
                self.proc.start()
                if self.window is not None:
                    self.window.set_visibility(True)
                    self.window.converge()
                if self.suspend:
                    self._signal_proc(signal.SIGCONT)

    # TODO(mv): hook this up to ProcController
    def _handle_respawn(self):
        if self.window is not None and self.state != ProcessState.STOPPED:
            self.window.converge()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
