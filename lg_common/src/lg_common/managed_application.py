import os
import signal
import threading

import rospy
from appctl_support import ProcController
from lg_common.msg import ApplicationState


class ManagedApplication(object):
    def __init__(self, cmd, window=None):
        self.cmd = cmd
        self.window = window
        self.state = ApplicationState.STOPPED

        self.lock = threading.Lock()
        self.proc = ProcController(cmd)
        self.sig_retry_timer = None

        rospy.on_shutdown(self._cleanup)

    def _cleanup(self):
        # explicit SIGCONT needed to prevent undeath
        self._signal_proc(signal.SIGCONT, retry=False)

    # TODO(mv): better pid retrieval and/or signalling as a feature of
    #           ProcController.
    def _signal_proc(self, sig, retry=True):
        if self.sig_retry_timer is not None:
            self.sig_retry_timer.shutdown()
        if self.proc.watcher is not None and self.proc.watcher.proc is not None:
            pid = self.proc.watcher.proc.pid
            try:
                os.kill(pid, sig)
                print "Sent signal {} to pid {}".format(sig, pid)
            except OSError:
                print "OSError sending signal {} to pid {}".format(sig, pid)
        else:
            print "Can't signal, no proc"
            if not retry:
                return

            def retry_signal(ev):
                self._signal_proc(sig)
            self.sig_retry_timer = rospy.Timer(
                rospy.Duration(0.1), retry_signal, oneshot=True
            )

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

            if state == ApplicationState.STOPPED:
                print "STOPPED"
                self._signal_proc(signal.SIGCONT, retry=False)
                self.proc.stop()

            elif state == ApplicationState.SUSPENDED:
                print "SUSPENDED"
                self.proc.start()
                self._signal_proc(signal.SIGSTOP)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()

            elif state == ApplicationState.HIDDEN:
                print "HIDDEN"
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                else:
                    rospy.logwarn(
                        'Tried to hide a ManagedApplication ' + \
                        'without a ManagedWindow'
                    )

            elif state == ApplicationState.VISIBLE:
                print "VISIBLE"
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(True)
                    self.window.converge()

    # TODO(mv): hook this up to ProcController
    def _handle_respawn(self):
        if self.window is not None and self.state != ApplicationState.STOPPED:
            self.window.converge()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
