"""Monotonic wall-time accounting; interrupted operations retain elapsed time."""
from contextlib import contextmanager
import time
import threading


class Timings:
    def __init__(self):
        self._lock = threading.Lock()
        self.seconds = {}
        self.calls = {}

    @contextmanager
    def measure(self, name):
        started = time.monotonic()
        with self._lock:
            self.calls[name] = self.calls.get(name, 0) + 1
        try:
            yield
        finally:
            with self._lock:
                self.seconds[name] = self.seconds.get(name, 0.0) + time.monotonic() - started

    def total(self):
        with self._lock:
            return sum(self.seconds.values())

    def snapshot(self):
        with self._lock:
            return {name: {'seconds': seconds, 'calls': self.calls[name]}
                    for name, seconds in self.seconds.items()}
