import threading


class Future:
    __slots__ = ("_event", "_result", "_error")

    def __init__(self):
        self._event = threading.Event()
        self._result = None
        self._error = None

    @property
    def result(self):
        return self._result

    def get(self):
        self._event.wait()
        if self._error:
            raise self._error
        return self._result

    def join(self):
        self._event.wait()
        if self._error:
            raise self._error
        return self._result

    def _set(self, result, error=None):
        self._result = result
        self._error = error
        self._event.set()
