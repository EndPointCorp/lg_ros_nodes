import logging
import os
import subprocess
import time
from logging.handlers import RotatingFileHandler

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
        self.mutex_timeout_period = rospy.get_param(
            '~mutex_timeout_period', 5)
        self.instance = rospy.get_name()
        self.mutex_event_logger = self._make_mutex_event_logger()
        rospy.Timer(rospy.Duration(10), self.keep_alive, oneshot=True)
        # only restart when worked is true, otherwise
        # it may have never worked
        self.worked = False

    @staticmethod
    def _main_thread_wait_state(pid):
        """Return Earth's main-thread kernel wait state, if available.

        A healthy Google Earth main thread normally waits in poll(). During the
        observed render freezes it remained in FUTEX_WAIT_PRIVATE while its
        KML and X11 helper threads were still alive. Reading wchan is cheap and
        lets us detect that state before the KML socket eventually disappears.
        """
        try:
            with open('/proc/{}/task/{}/wchan'.format(pid, pid), 'r') as handle:
                return handle.read().strip()
        except (IOError, OSError):
            # The process may have exited between obtaining its PID and reading
            # procfs. ProcController will handle that normal respawn race.
            return None

    @staticmethod
    def _main_thread_syscall(pid):
        """Return the raw main-thread syscall snapshot for diagnostics."""
        try:
            with open('/proc/{}/task/{}/syscall'.format(pid, pid), 'r') as handle:
                return handle.read().strip()
        except (IOError, OSError):
            return 'unavailable'

    def _make_mutex_event_logger(self):
        """Create a persistent, per-instance rotating mutex event log."""
        log_dir = rospy.get_param(
            '~mutex_log_dir', '/home/lg/.ros/earth_mutex')
        instance_slug = self.instance.strip('/').replace('/', '_') or 'unnamed'
        try:
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, instance_slug + '.log')
            event_logger = logging.getLogger(
                '{}:earth_mutex_events'.format(self.instance))
            event_logger.setLevel(logging.INFO)
            event_logger.propagate = False
            if not event_logger.handlers:
                handler = RotatingFileHandler(
                    log_path, maxBytes=1024 * 1024, backupCount=3)
                handler.setFormatter(logging.Formatter(
                    '%(asctime)s %(levelname)s %(message)s'))
                event_logger.addHandler(handler)
            logger.info(
                'Earth mutex event log instance={} path={}'
                .format(self.instance, log_path))
            return event_logger
        except (IOError, OSError) as e:
            logger.error(
                'Unable to create Earth mutex event log instance={} dir={}: {}'
                .format(self.instance, log_dir, e))
            return None

    def _log_mutex_event(self, level, event, pid, wchan, samples,
                         elapsed, syscall='unavailable', action='none'):
        message = (
            'event={} instance={} pid={} wchan={} samples={} '
            'elapsed_sec={:.1f} syscall="{}" action={}'
            .format(event, self.instance, pid, wchan or 'unavailable', samples,
                    elapsed, syscall.replace('"', "'"), action))
        getattr(logger, level)(message)
        if self.mutex_event_logger is not None:
            getattr(self.mutex_event_logger, level)(message)

    def keep_alive(self, *args, **kwargs):
        logger.debug("just in first keep_alive")
        loop_timeout = 1
        counter = 0
        mutex_counter = 0
        mutex_started_at = None
        mutex_wait_state = None
        last_pid = None
        rospy.sleep(1)
        while not rospy.is_shutdown():
            try:
                pid = self.earth_proc.proc.watcher.proc.pid
            except AttributeError as e:
                counter = 0
                logger.warning("Earth proc doesn't exist {}".format(e))
                rospy.sleep(loop_timeout)
                continue
            if pid != last_pid:
                if mutex_counter > 0:
                    self._log_mutex_event(
                        'info', 'earth_mutex_wait_ended', last_pid,
                        mutex_wait_state, mutex_counter,
                        time.monotonic() - mutex_started_at,
                        action='process_changed')
                counter = 0
                mutex_counter = 0
                mutex_started_at = None
                mutex_wait_state = None
                last_pid = pid
            cmd = "lsof -Pn -p {} -a -i @localhost:8765".format(pid).split(' ')
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
                    logger.info("RELAUNCHING worked: {}  counter: {}".format(self.worked, counter))
                    self.earth_proc.handle_soft_relaunch()
                    counter = 0
                    mutex_counter = 0
                    mutex_started_at = None
                    mutex_wait_state = None
                    self.worked = False
                    rospy.sleep(loop_timeout)
                    continue

            wait_state = self._main_thread_wait_state(pid)
            if (self.mutex_timeout_period > 0 and self.worked and wait_state and
                    wait_state.startswith('futex_wait')):
                if mutex_counter == 0:
                    mutex_started_at = time.monotonic()
                    mutex_wait_state = wait_state
                    self._log_mutex_event(
                        'warning', 'earth_mutex_wait_detected', pid, wait_state,
                        1, 0.0, self._main_thread_syscall(pid))
                mutex_counter += 1
                if mutex_counter >= self.mutex_timeout_period:
                    elapsed = time.monotonic() - mutex_started_at
                    self._log_mutex_event(
                        'error', 'earth_mutex_wait_confirmed', pid, wait_state,
                        mutex_counter, elapsed, self._main_thread_syscall(pid),
                        action='soft_relaunch')
                    self.earth_proc.handle_soft_relaunch()
                    counter = 0
                    mutex_counter = 0
                    mutex_started_at = None
                    mutex_wait_state = None
                    self.worked = False
            else:
                if mutex_counter > 0:
                    self._log_mutex_event(
                        'info', 'earth_mutex_wait_recovered', pid,
                        mutex_wait_state, mutex_counter,
                        time.monotonic() - mutex_started_at,
                        action='none')
                mutex_counter = 0
                mutex_started_at = None
                mutex_wait_state = None
            rospy.sleep(loop_timeout)
