import threading

from .error import RPCError


def _raise_wire_error(err):
    # Server-side errors arrive as raw msgpack values (bytes/str/list/dict), not
    # exceptions — raising them directly yields "exceptions must derive from
    # BaseException". Pass through real exceptions; wrap everything else.
    if isinstance(err, BaseException):
        raise err
    raise RPCError(err)


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
            _raise_wire_error(self._error)
        return self._result

    def join(self):
        self._event.wait()
        if self._error:
            _raise_wire_error(self._error)
        return self._result

    def _set(self, result, error=None):
        self._result = result
        self._error = error
        self._event.set()
