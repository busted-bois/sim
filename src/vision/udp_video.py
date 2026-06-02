"""UDP port 5600 chunked JPEG reassembly with 24-byte LE metadata header."""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass, field

# Little-endian header: frame_id u32, chunk_index u16, chunk_total u16,
# sim_time_ns u64, payload_size u32, reserved u32
HEADER_STRUCT = struct.Struct("<IHHQII")
HEADER_SIZE = HEADER_STRUCT.size  # 24


@dataclass
class _PartialFrame:
    sim_time_ns: int
    chunk_total: int
    chunks: dict[int, bytes] = field(default_factory=dict)
    last_update_monotonic: float = 0.0


class UdpVideoReassembler:
    def __init__(self, *, frame_timeout_s: float = 0.2) -> None:
        self._timeout_s = max(0.05, float(frame_timeout_s))
        self._partials: dict[int, _PartialFrame] = {}
        self._lock = threading.Lock()
        self._frames_completed = 0
        self._packets_received = 0

    @property
    def frames_completed(self) -> int:
        with self._lock:
            return self._frames_completed

    @property
    def packets_received(self) -> int:
        with self._lock:
            return self._packets_received

    def ingest(self, packet: bytes) -> tuple[int, bytes] | None:
        if len(packet) < HEADER_SIZE:
            return None
        frame_id, chunk_index, chunk_total, sim_time_ns, payload_size, _reserved = (
            HEADER_STRUCT.unpack_from(packet, 0)
        )
        payload = packet[HEADER_SIZE : HEADER_SIZE + payload_size]
        if len(payload) != payload_size:
            return None

        with self._lock:
            self._packets_received += 1
            now = time.monotonic()
            partial = self._partials.get(frame_id)
            if partial is None or partial.chunk_total != chunk_total:
                partial = _PartialFrame(
                    sim_time_ns=int(sim_time_ns),
                    chunk_total=int(chunk_total),
                )
                self._partials[frame_id] = partial
            partial.chunks[int(chunk_index)] = payload
            partial.last_update_monotonic = now
            self._expire_old(now)
            if len(partial.chunks) < partial.chunk_total:
                return None
            if any(i not in partial.chunks for i in range(partial.chunk_total)):
                return None
            jpeg = b"".join(partial.chunks[i] for i in range(partial.chunk_total))
            del self._partials[frame_id]
            self._frames_completed += 1
            return int(sim_time_ns), jpeg

    def _expire_old(self, now: float) -> None:
        expired = [
            fid
            for fid, partial in self._partials.items()
            if now - partial.last_update_monotonic > self._timeout_s
        ]
        for fid in expired:
            del self._partials[fid]


class UdpVideoReceiver:
    def __init__(
        self,
        *,
        port: int = 5600,
        host: str = "0.0.0.0",
        frame_timeout_s: float = 0.2,
    ) -> None:
        self._port = int(port)
        self._host = host
        self._reassembler = UdpVideoReassembler(frame_timeout_s=frame_timeout_s)
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._callback = None
        self._latest_error: str | None = None

    def set_frame_callback(self, callback) -> None:
        self._callback = callback

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._loop, name="udp_video", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _loop(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                packet, _addr = self._sock.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError as exc:
                self._latest_error = str(exc)
                continue
            try:
                result = self._reassembler.ingest(packet)
                if result is not None and self._callback is not None:
                    self._callback(result[0], result[1])
            except Exception as exc:
                self._latest_error = str(exc)

    @property
    def stats(self) -> dict[str, int | str | None]:
        return {
            "packets": self._reassembler.packets_received,
            "frames": self._reassembler.frames_completed,
            "error": self._latest_error,
        }
