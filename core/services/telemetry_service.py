# src/MAVLink/telemetry_service.py
# Thread-safe telemetry polling with cached TelemetryDTO and zero-copy snapshot view.

import time
from typing import Any, Dict, Optional
import threading
from types import MappingProxyType  # read-only dict view

from core.services.mavlink_connection import MavlinkConnection
from core.dto.telemetry_dto import TelemetryDTO



class TelemetryService:
    def __init__(self, mav: MavlinkConnection, rate_hz: float = 10.0) -> None:
        self._mav = mav
        self._rate_hz = max(1.0, float(rate_hz))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Latest flat snapshot + read-only proxy + cached DTO
        self._snapshot: Dict[str, Any] = {}
        self._snapshot_proxy: MappingProxyType = MappingProxyType(self._snapshot)
        self._dto: TelemetryDTO = TelemetryDTO()  # immutable, safe to share
        self.old_time = time.time()

    # ---- lifecycle ----
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="TelemetryService", daemon=True)
        self._thread.start()

    def stop(self, join: bool = True, timeout: float = 1.0) -> None:
        self._stop.set()
        if join and self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)

    # ---- background polling ----
    def _loop(self) -> None:
        period = 1.0 / self._rate_hz
        while not self._stop.is_set():
            if not self._mav.connect_status_flag:
                time.sleep(0.2)
                self._dto = TelemetryDTO()
                continue
            t0 = time.perf_counter()
            try:
                # MAVLink returns dict[key] = ('new'|'old', value)
                data = self._mav.read_data_from_uav() or {}
                # print("alt_baro", data.get("alt_baro"))
                flat = {k: v for k, (_, v) in data.items()}
                dto = TelemetryDTO.from_snapshot(flat)  # build ONCE per poll

                with self._lock:
                    # swap all atomically
                    self._snapshot = flat
                    self._snapshot_proxy = MappingProxyType(self._snapshot)
                    new_time = time.time()
                    # print(new_time - self.old_time)
                    self.old_time = new_time
                    self._dto = dto
            except Exception as e:
                # swallow transient errors
                print("Telemetry service thread, exception occured", e)
                raise e

            dt = time.perf_counter() - t0
            if dt < period:
                pass
                # time.sleep(period - dt)  # Comment for the smooth work

    # ---- consumer API ----
    def dto(self) -> TelemetryDTO:
        """Return the latest cached DTO (immutable, safe to share across threads)."""
        with self._lock:
            return self._dto

    def snapshot(self) -> Dict[str, Any]:
        """Return a shallow COPY of the latest flat snapshot."""
        with self._lock:
            return dict(self._snapshot)

    def snapshot_view(self) -> MappingProxyType:
        """Return a ZERO-COPY read-only view of the latest flat snapshot."""
        with self._lock:
            return self._snapshot_proxy

    def get(self, key: str, default: Any = None) -> Any:
        with self._lock:
            return self._snapshot.get(key, default)

    def set_rate(self, rate_hz: float) -> None:
        self._rate_hz = max(1.0, float(rate_hz))

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())
