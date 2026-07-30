#!/usr/bin/env python3
"""Publish X11 keyboard and mouse input as an lg_activity source."""

import logging
import signal
import time


NODE_NAME = 'kb_mouse_activity'
ACTIVITY_TOPIC = '/kb_mouse_activity'
MIN_PUBLISH_INTERVAL_SECONDS = 0.25

logger = logging.getLogger(NODE_NAME)


class ActivityLimiter:
    """Limit high-rate pointer motion to a useful activity heartbeat."""

    def __init__(self, interval=MIN_PUBLISH_INTERVAL_SECONDS):
        self.interval = interval
        self.last_publish = float('-inf')

    def ready(self, now):
        if now - self.last_publish < self.interval:
            return False
        self.last_publish = now
        return True


def is_activity_event(event_type, xlib_constants):
    return event_type in {
        xlib_constants.KeyPress,
        xlib_constants.ButtonPress,
        xlib_constants.MotionNotify,
    }


class XRecordActivityMonitor:
    """Observe X11 input with RECORD without grabbing the input devices."""

    def __init__(self, publish):
        from Xlib import X, display
        from Xlib.ext import record
        from Xlib.protocol import rq

        self.X = X
        self.record = record
        self.rq = rq
        self.publish = publish
        self.limiter = ActivityLimiter()
        self.control_display = display.Display()
        self.record_display = display.Display()
        self.context = None
        self.stopping = False
        self.publish_count = 0

        if not self.record_display.has_extension('RECORD'):
            raise RuntimeError('X11 RECORD extension is unavailable')

    def handle_reply(self, reply):
        if (
            self.stopping
            or reply.category != self.record.FromServer
            or reply.client_swapped
            or not reply.data
        ):
            return

        data = reply.data
        while data:
            event, data = self.rq.EventField(None).parse_binary_value(
                data,
                self.record_display.display,
                None,
                None,
            )
            if not is_activity_event(event.type, self.X):
                continue
            if self.limiter.ready(time.monotonic()):
                self.publish()
                self.publish_count += 1
                if self.publish_count == 1:
                    logger.info('published first keyboard/mouse activity event')

    def stop(self, _signum=0, _frame=None):
        if self.stopping:
            return
        self.stopping = True
        if self.context is not None:
            try:
                self.control_display.record_disable_context(self.context)
                self.control_display.flush()
            except Exception:
                logger.debug(
                    'failed to disable X11 RECORD context',
                    exc_info=True,
                )

    def run(self):
        self.context = self.record_display.record_create_context(
            0,
            [self.record.AllClients],
            [
                {
                    'core_requests': (0, 0),
                    'core_replies': (0, 0),
                    'ext_requests': (0, 0, 0, 0),
                    'ext_replies': (0, 0, 0, 0),
                    'delivered_events': (0, 0),
                    'device_events': (
                        self.X.KeyPress,
                        self.X.MotionNotify,
                    ),
                    'errors': (0, 0),
                    'client_started': False,
                    'client_died': False,
                }
            ],
        )
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        logger.info(
            'monitoring X11 keyboard/mouse activity display=%s topic=%s',
            self.record_display.get_display_name(),
            ACTIVITY_TOPIC,
        )
        try:
            self.record_display.record_enable_context(
                self.context,
                self.handle_reply,
            )
        finally:
            try:
                self.record_display.record_free_context(self.context)
            except Exception:
                logger.debug(
                    'failed to free X11 RECORD context',
                    exc_info=True,
                )
            self.record_display.close()
            self.control_display.close()


def main():
    import rospy
    from std_msgs.msg import Bool

    logging.basicConfig(level=logging.INFO)
    rospy.init_node(NODE_NAME, anonymous=False)
    publisher = rospy.Publisher(ACTIVITY_TOPIC, Bool, queue_size=1)
    monitor = XRecordActivityMonitor(
        lambda: publisher.publish(Bool(data=True)),
    )
    monitor.run()


if __name__ == '__main__':
    main()
