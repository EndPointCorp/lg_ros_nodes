import os
import signal
import threading
import time

import rospy
from appctl_support import ProcController
from lg_msg_defs.msg import ApplicationState
from lg_common.helpers import is_valid_state

from lg_common.logger import get_logger
logger = get_logger('managed_application')

SIG_RETRY_DELAY = 0.05


class ManagedApplication(object):
    def __init__(self, cmd, shell=False, window=None, initial_state=None, respawn=True, env=None,
                 stdout=None, stderr=None):
        """
        respawn handles whether or not the application shall be automatically
                respawned at all, default is True.

        """
        self.cmd = cmd
        self.window = window
        self.lock = threading.Lock()
        self.proc = ProcController(cmd,
                                   spawn_hooks=[self._handle_respawn],
                                   shell=shell,
                                   respawn=respawn,
                                   env=env,
                                   stdout=stdout,
                                   stderr=stderr)

        self._respawn_handlers = []
        self._state_handlers = []

        if initial_state and is_valid_state(initial_state):
            logger.debug('setting initial state to %s' % initial_state)
            self.state = initial_state
            self.set_state(self.state)
        else:
            self.state = ApplicationState.STOPPED

        self.post_init()

    def close(self):
        """
        End this ManagedApplication immediately.  After closing, the instance
        can no longer be used.
        """
        self.set_state(ApplicationState.STOPPED)
        del self.lock
        del self.cmd
        del self.state
        del self.proc
        del self.window
        del self._respawn_handlers[:]
        del self._state_handlers[:]

    def post_init(self):
        pass

    def add_respawn_handler(self, handler):
        self._respawn_handlers.append(handler)

    def add_state_handler(self, handler):
        self._state_handlers.append(handler)

    def __str__(self):
        string_representation = "<ManagedApplication: state: %s, window: %s, cmd: %s, proc: %s" % (self.state, self.window, self.cmd, self.proc)
        return string_representation

    def __repr__(self):
        return self.__str__()

    def get_state(self):
        with self.lock:
            return self.state

    def set_state(self, state):
        with self.lock:
            state_changed = False
            if state != self.state:
                state_changed = True
            self.state = state

            if state == ApplicationState.STOPPED:
                logger.debug("STOPPED")
                self.proc.stop()
                if self.window is not None:
                    self.window.set_visibility(False)

            elif state == ApplicationState.SUSPENDED:
                logger.debug("SUSPENDED")
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                self.proc.start()

            elif state == ApplicationState.HIDDEN:
                logger.debug("HIDDEN")
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                else:
                    logger.warning(
                        'Tried to hide a ManagedApplication ' +
                        'without a ManagedWindow'
                    )
                self.proc.start()

            elif state == ApplicationState.STARTED:
                logger.debug("STARTED")
                if self.window is not None:
                    self.window.set_visibility(False)
                    self.window.converge()
                self.proc.start()

            elif state == ApplicationState.VISIBLE:
                logger.debug("VISIBLE")
                if self.window is not None:
                    self.window.set_visibility(True)
                    self.window.converge()
                self.proc.start()

            def run_handler(handler):
                try:
                    handler(state)
                except Exception:
                    logger.exception('Exception in a state change handler')

            list(map(run_handler, self._state_handlers))

    def handle_state_msg(self, msg):
        logger.debug('Got state message: {}'.format(msg))
        self.set_state(msg.state)

    # TODO(mv): hook this up to ProcController
    def _handle_respawn(self):
        def run_handler(handler):
            try:
                handler()
            except Exception:
                logger.exception('Exception in a respawn handler')

        list(map(run_handler, self._respawn_handlers))

    def handle_soft_relaunch(self, *args, **kwargs):
        logger.debug('managed application relaunch...')
        self.proc.handle_soft_relaunch()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
