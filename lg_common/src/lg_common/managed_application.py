import os
import signal
import threading

import rospy
from appctl_support import ProcController
from lg_common.msg import ApplicationState

SIG_RETRY_DELAY = 0.05


class ManagedApplication(object):
    """Provides a combination of process state control and window management.

    A ManagedApplication is always initialized to an inert state. No processes
    are spawned or windows moved until its state is changed.
    """

    def __init__(self, cmd, window=None):
        """Creates a ManagedApplication instance.

        Args:
            cmd (List[str]): The command to run as a managed process.
            window (Optional[ManagedWindow]): A window to manage along with
                the process.
        """
        self.cmd = cmd
        self.window = window
        self.state = ApplicationState.STOPPED

        self.lock = threading.RLock()
        self.proc = ProcController(cmd)
        self.sig_retry_timer = None

        rospy.on_shutdown(self._cleanup)

    def _cleanup(self):
        """Clean up the instance for shutdown.

        This is intended to be called upon `rospy` shutdown.
        """
        # explicit SIGCONT needed to prevent undeath
        self._signal_proc(signal.SIGCONT, retry=False)

    def _signal_proc(self, sig, retry=True):
        """Send a signal to the managed process.

        This method is presently disabled and may be considered deprecated.

        TODO(mv): better pid retrieval and/or signalling as a feature of
            ProcController.

        Args:
            sig (int): signum to send to the process.
            retry (Optional[bool]): If the process is not available, schedule
                a retry. Any existing retries will be cancelled.
        """
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
        """Return the current goal state."""
        with self.lock:
            return self.state

    def set_state(self, state):
        """Set the goal state.

        This method only takes action if the goal state is different from the
        previous goal state.

        Args:
            state (ApplicationState): Desired state for this instance.
        """
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
            self.window.converge()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
