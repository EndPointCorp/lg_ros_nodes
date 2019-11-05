from .query_notifier import QueryNotifier
from collections import deque
import threading


class QueryQueue(object):
    def __init__(self, query_path, maxlen=10):
        self.query_path = query_path
        self.lock = threading.Lock()
        self.waiting = False
        self.q = deque(maxlen=maxlen)
        self.notifier = QueryNotifier(self.query_path)
        self.notifier.add_delete_handler(self.handle_consumed_query)
        self.notifier.start()

    def stop(self):
        self.notifier.stop()

    def clear(self):
        with self.lock:
            self.q.clear()
            self.waiting = False

    def handle_consumed_query(self):
        with self.lock:
            if len(self.q) > 0:
                self._write_query(self.q.pop())
            else:
                self.waiting = False

    def post_query(self, query):
        with self.lock:
            if self.waiting:
                self.q.appendleft(query)
            else:
                self._write_query(query)

    def _write_query(self, query):
        self.waiting = True
        with open(self.query_path, 'w') as f:
            f.write(query)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
