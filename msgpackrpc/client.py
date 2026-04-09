import builtins
import socket
import threading

import msgpack

from .error import RPCError, TimeoutError, TransportError
from .future import Future
from .message import REQUEST, RESPONSE


class Client:
    def __init__(
        self, address, timeout=3600, pack_encoding="utf-8", unpack_encoding=None, **kwargs
    ):
        self._address = address
        self._timeout = timeout
        self._msgid = 0
        self._lock = threading.Lock()
        self._pending = {}
        self._socket = None
        self._running = False
        self._thread = None

        # Build pack kwargs — handle both msgpack 0.5.x (encoding) and 1.0+ (no encoding)
        self._pack_kwargs = {"default": lambda x: x.to_msgpack()}
        if pack_encoding:
            try:
                msgpack.packb(b"test", encoding=pack_encoding)
                self._pack_kwargs["encoding"] = pack_encoding
            except TypeError:
                pass

        # Build unpack kwargs
        self._unpack_kwargs = {}
        if unpack_encoding:
            try:
                msgpack.unpackb(msgpack.packb(b"test"), encoding=unpack_encoding)
                self._unpack_kwargs["encoding"] = unpack_encoding
            except TypeError:
                pass

        self._connect()
        self._start_reader()

    def _connect(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(self._timeout)
        try:
            self._socket.connect((self._address.host, self._address.port))
        except builtins.TimeoutError:
            raise TimeoutError(f"Connection to {self._address.host}:{self._address.port} timed out")
        except OSError as e:
            raise TransportError(
                f"Failed to connect to {self._address.host}:{self._address.port}: {e}"
            )
        self._socket.settimeout(None)

    def _start_reader(self):
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        # Create unpacker with version-appropriate kwargs
        try:
            unpacker = msgpack.Unpacker(**self._unpack_kwargs)
        except TypeError:
            unpacker = msgpack.Unpacker()

        try:
            while self._running:
                data = self._socket.recv(4096)
                if not data:
                    break
                unpacker.feed(data)
                for msg in unpacker:
                    if msg[0] == RESPONSE:
                        _, msgid, error, result = msg[0], msg[1], msg[2], msg[3]
                        future = self._pending.pop(msgid, None)
                        if future is not None:
                            future._set(result, error)
        except (OSError, ConnectionError):
            pass
        finally:
            self._running = False

    def _next_msgid(self):
        with self._lock:
            msgid = self._msgid
            self._msgid += 1
        return msgid

    def _send(self, method, args):
        msgid = self._next_msgid()
        future = Future()
        self._pending[msgid] = future
        payload = msgpack.packb([REQUEST, msgid, method, list(args)], **self._pack_kwargs)
        try:
            self._socket.sendall(payload)
        except OSError as e:
            self._pending.pop(msgid, None)
            raise TransportError(f"Failed to send request: {e}")
        return future

    def call(self, method, *args):
        future = self._send(method, args)
        future._event.wait(timeout=self._timeout)
        if not future._event.is_set():
            self._pending = {k: v for k, v in self._pending.items() if v is not future}
            raise TimeoutError(f"RPC call '{method}' timed out after {self._timeout}s")
        if future._error:
            raise RPCError(future._error)
        return future._result

    def call_async(self, method, *args):
        return self._send(method, args)

    def close(self):
        self._running = False
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
