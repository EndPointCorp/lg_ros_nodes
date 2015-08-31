import os
import signal
import threading

import rospy
from appctl_support import ProcController
from lg_common.msg import ApplicationState

SIG_RETRY_DELAY = 0.05


class ManagedApplication(object):
    def __init__(self, cmd, window=None):
        self.cmd = cmd
        self.window = window
        self.state = ApplicationState.STOPPED

        self.lock = threading.RLock()
        self.proc = ProcController(cmd, spawn_hooks=[self._handle_respawn])
        self.sig_retry_timer = None

        rospy.on_shutdown(self._cleanup)

    def __str__(self):
        string_representation = "<ManagedApplication: state: %s, window: %s, cmd: %s, proc: %s" % (self.state, self.window, self.cmd, self.proc)
        return string_representation

    def __repr__(self):
        representation = "<ManagedApplication: state: %s, window: %s, cmd: %s, proc: %s" % (self.state, self.window, self.cmd, self.proc)
        return representation


    def _cleanup(self):
        # explicit SIGCONT needed to prevent undeath
        self._signal_proc(signal.SIGCONT, retry=False)

    # TODO(mv): better pid retrieval and/or signalling as a feature of
    #           ProcController.
    def _signal_proc(self, sig, retry=True):
        rospy.logwarn('_signal_proc disabled')
        return

        if self.sig_retry_timer is not None:
            self.sig_retry_timer.shutdown()

        if self.proc.watcher is not None and \
           self.proc.watcher.proc is not None:

            pid = self.proc.watcher.proc.pid
            try:
                os.kill(pid, sig)
                rospy.loginfo("Sent signal {} to pid {}".format(sig, pid))
            except OSError:
                rospy.logerr(
                    "OSError sending signal {} to pid {}".format(sig, pid))
        else:
            rospy.logwarn("Can't signal, no proc")
            if not retry:
                return

            def retry_signal(ev):
                self._signal_proc(sig)
            self.sig_retry_timer = rospy.Timer(
                rospy.Duration(SIG_RETRY_DELAY), retry_signal, oneshot=True
            )

    def get_state(self):
        with self.lock:
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
                rospy.loginfo("STOPPED")
                self._signal_proc(signal.SIGCONT, retry=False)
                self.proc.stop()
                if self.window is not None:
                    self.window.set_visibility(False)

            elif state == ApplicationState.SUSPENDED:
                rospy.loginfo("SUSPENDED")
                self.proc.start()
                self._signal_proc(signal.SIGSTOP)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()

            elif state == ApplicationState.HIDDEN:
                rospy.loginfo("HIDDEN")
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                else:
                    rospy.logwarn(
                        'Tried to hide a ManagedApplication ' +
                        'without a ManagedWindow'
                    )

            elif state == ApplicationState.VISIBLE:
                rospy.loginfo("VISIBLE")
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(True)
                    self.window.converge()

    def handle_state_msg(self, msg):
        rospy.loginfo('Got state message: {}'.format(msg))
        with self.lock:
            self.set_state(msg.state)

    # TODO(mv): hook this up to ProcController
    def _handle_respawn(self):
        if self.window is not None and self.state != ApplicationState.STOPPED:
            rospy.loginfo("Handling unwanted respawn of %s by converging the window" % self)
            self.window.converge()
        if (self.window is None) and (ApplicationState.STOPPED):
            rospy.loginfo("Handling unwanted respawn of %s by killing the process" % self)
            self.proc.stop()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
