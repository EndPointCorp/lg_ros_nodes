import rospy
import os
import pyinotify
import sys
from collections.abc import Callable
from lg_common.logger import get_logger
logger = get_logger('query_notifier')


class _QueryDeleteProcessor(pyinotify.ProcessEvent):
    def process_IN_DELETE(self, event):
        self.__class__.handle_delete(event)

    def process_default(self, event):
        pass


class QueryNotifier():
    def __init__(self, query_path):
        self.query_path = query_path
        _QueryDeleteProcessor.handle_delete = self.handle_delete
        self.delete_handlers = []
        self.wm = pyinotify.WatchManager()
        self.notifier = pyinotify.ThreadedNotifier(self.wm)
        self.wm.watch_transient_file(
            filename=self.query_path,
            mask=pyinotify.ALL_EVENTS,
            proc_class=_QueryDeleteProcessor
        )

    def add_delete_handler(self, handler):
        assert isinstance(handler, Callable)
        self.delete_handlers.append(handler)

    def handle_delete(self, *args, **kwargs):
        def run_handler(h):
            try:
                h()
            except Exception as e:
                logger.error('Caught an Exception in delete handler!')
                logger.error(sys.exc_info()[2])
        list(map(run_handler, self.delete_handlers))

    def start(self):
        self.notifier.start()

    def stop(self):
        self.notifier.stop()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
