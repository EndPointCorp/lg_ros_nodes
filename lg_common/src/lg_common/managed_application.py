import os
import signal
import threading

import rospy
from appctl_support import ProcController
from lg_common.msg import ApplicationState
from lg_common.helpers import is_valid_state

SIG_RETRY_DELAY = 0.05


class ManagedApplication(object):
    def __init__(self, cmd, shell=False, window=None, initial_state=None, respawn=True):
        """
        respawn handles whether or not the application shall be automatically
                respawned at all, default is True.

        """
        self.cmd = cmd
        self.window = window
        self.lock = threading.RLock()
        self.proc = ProcController(cmd,
                                   spawn_hooks=[self._handle_respawn],
                                   shell=shell,
                                   respawn=respawn)
        self.sig_retry_timer = None

        if initial_state and is_valid_state(initial_state):
            rospy.logdebug('setting initial state to %s' % initial_state)
            self.state = initial_state
            self.set_state(self.state)
        else:
            self.state = ApplicationState.STOPPED

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
        # currently disabled (and for the foreseeable future)
        return

        if self.sig_retry_timer is not None:
            self.sig_retry_timer.shutdown()

        if self.proc.watcher is not None and \
           self.proc.watcher.proc is not None:

            pid = self.proc.watcher.proc.pid
            try:
                os.kill(pid, sig)
                rospy.logdebug("Sent signal {} to pid {}".format(sig, pid))
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

            # if not state_changed:
            #    return

            if state == ApplicationState.STOPPED:
                rospy.logdebug("STOPPED")
                self._signal_proc(signal.SIGCONT, retry=False)
                self.proc.stop()
                if self.window is not None:
                    self.window.set_visibility(False)

            elif state == ApplicationState.SUSPENDED:
                rospy.logdebug("SUSPENDED")
                self.proc.start()
                self._signal_proc(signal.SIGSTOP)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()

            elif state == ApplicationState.HIDDEN:
                rospy.logdebug("HIDDEN")
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

            elif state == ApplicationState.STARTED:
                rospy.loginfo("STARTED")
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()

            elif state == ApplicationState.VISIBLE:
                rospy.logdebug("VISIBLE")
                self.proc.start()
                self._signal_proc(signal.SIGCONT)
                if self.window is not None:
                    self.window.set_visibility(True)
                    self.window.converge()

    def handle_state_msg(self, msg):
        rospy.logdebug('Got state message: {}'.format(msg))
        with self.lock:
            self.set_state(msg.state)

    # TODO(mv): hook this up to ProcController
    def _handle_respawn(self):
        if (self.window is not None) and (self.state != ApplicationState.STOPPED):
            rospy.logdebug("Handling unwanted respawn of %s by converging the window" % self)
            self.window.converge()
        if (self.window is None) and (self.state == ApplicationState.STOPPED):
            rospy.logdebug("Handling unwanted respawn of %s by killing the process" % self)
            self.proc.stop()
        self.set_state(self.state)

    def handle_soft_relaunch(self, *args, **kwargs):
        rospy.logdebug('managed application relaunch...')
        self.proc.handle_soft_relaunch()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
