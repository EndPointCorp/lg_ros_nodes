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
                query, _coalesce_key = self.q.pop()
                self._write_query(query)
            else:
                self.waiting = False

    def post_query(self, query, coalesce_key=None):
        with self.lock:
            if self.waiting:
                if coalesce_key is not None:
                    # A continuous absolute-view gesture only needs its newest
                    # target; keep unrelated searches, tours, and planet calls.
                    self.q = deque(
                        [
                            (queued_query, queued_key)
                            for queued_query, queued_key in self.q
                            if queued_key != coalesce_key
                        ],
                        maxlen=self.q.maxlen,
                    )
                self.q.appendleft((query, coalesce_key))
            else:
                self._write_query(query)

    def _write_query(self, query):
        self.waiting = True
        with open(self.query_path, 'w') as f:
            f.write(query)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
