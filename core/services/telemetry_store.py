# All comments in English.
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional
from collections import defaultdict, deque
from datetime import datetime, timezone
import threading

# ----------------------------
# Data model
# ----------------------------

@dataclass(frozen=True)
class MsgRecord:
    """
    Single MAVLink message snapshot for a given TYPE.
    - ver: monotonic per-type version (increments every time THIS type arrives)
    - time: human-readable UTC timestamp (ISO-8601, milliseconds)
    - data: raw message dict (cleaned: without 'mavpackettype' by default)
    """
    type: str
    ver: int
    time: str
    data: Dict[str, Any]

    def to_grouped(self) -> Dict[str, Any]:
        """Convert to grouped JSON-friendly structure."""
        return {"time": self.time, "ver": self.ver, "data": self.data}


# ----------------------------
# Store
# ----------------------------

class TelemetryStore:
    """
    Schema-less telemetry store.

    Key points:
      - Per-type versioning: each MAVLink TYPE has its own version counter.
      - Keeps 'last' record per type and an optional ring-buffer 'history' per type.
      - Thread-safe ingest() and queries.
      - Snapshot returns: { "TYPE": { "time": "...", "ver": N, "data": {...} }, ... }

    Notes:
      * 'version' here is NOT global; it is specific to a message TYPE.
      * If you need incremental polling, the client can send a map {TYPE: last_seen_ver}
        and call snapshot_since() to get only updated types.
    """

    def __init__(self, keep_history: bool = True, history_size: int = 50):
        """
        :param keep_history: enable per-type ring-buffer history
        :param history_size: default history length per message type
        :param strip_mavpackettype: remove 'mavpackettype' field from stored data
        """
        self._lock = threading.Lock()
        self._keep_history = keep_history
        self._history_size = max(1, int(history_size))

        # Per-type state
        self._versions_by_type: Dict[str, int] = defaultdict(int)            # TYPE -> last version
        self._last_by_type: Dict[str, MsgRecord] = {}                        # TYPE -> last MsgRecord
        self._history: Dict[str, deque[MsgRecord]] = defaultdict(            # TYPE -> ring buffer
            lambda: deque(maxlen=self._history_size)
        ) if keep_history else {}

        self._types_seen: set[str] = set()

    # -------- Utilities --------

    @staticmethod
    def _now_iso_utc() -> str:
        """Return human-readable UTC timestamp (ISO-8601, milliseconds)."""
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    
    def _normalize_float(self, x: float) -> Any:
        """Apply NaN/Inf policy."""
        if math.isfinite(x):
            return x
        return None
    

    def _sanitize(self, v: Any) -> Any:
        """Recursive method used only for cleaning packages"""
        if isinstance(v, dict):
            return {k: self._sanitize(x) for k, x in v.items() if k != "mavpackettype"}
        if isinstance(v, (list, tuple)):
            return [self._sanitize(x) for x in v]
        if isinstance(v, (bytes, bytearray, memoryview)):
            return list(v)  # or v.hex()
        if isinstance(v, float):
            return self._normalize_float(v)
        return v

    def _clean_dict(self, d: dict[str, Any]) -> dict[str, Any]:
        """Removes redundant messages."""
        out = dict(d)
        msg_type: str = out.pop("mavpackettype", "")
        if msg_type.upper().startswith("UNKNOWN"):
            print("delete", d)
            out = {}
        
        return self._sanitize(out)

    # -------- Ingest --------

    def ingest(self, msg_type: str, msg_dict: Dict[str, Any]) -> None:
        """
        Ingest ANY MAVLink message.
        - Increments version for this TYPE only.
        - Builds 'last' record and (optionally) appends to history ring-buffer.
        Thread-safe.
        """
        payload = self._clean_dict(msg_dict)
        tstr = self._now_iso_utc()

        with self._lock:
            # bump per-type version
            self._versions_by_type[msg_type] += 1
            ver = self._versions_by_type[msg_type]

            rec = MsgRecord(type=msg_type, ver=ver, time=tstr, data=payload)
            self._last_by_type[msg_type] = rec
            self._types_seen.add(msg_type)

            if self._keep_history:
                self._history[msg_type].append(rec)

    # -------- Queries (thread-safe) --------

    def get_types(self) -> List[str]:
        """Return a sorted list of message types seen so far."""
        with self._lock:
            return sorted(self._types_seen)

    def get_last(self, msg_type: str) -> Dict[str, Any]:
        """
        Return the last record for a TYPE as grouped dict:
          { "time": "...", "ver": N, "data": {...} }
        """
        with self._lock:
            rec = self._last_by_type.get(msg_type)
            return rec.to_grouped() if rec else {}

    def get_history(self, msg_type: str, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Return last 'limit' history items for a TYPE (grouped dicts).
        If history disabled or TYPE unknown, returns [].
        """
        with self._lock:
            if not self._keep_history or msg_type not in self._history:
                return []
            hist = self._history[msg_type]
            if limit is None or limit >= len(hist):
                return [r.to_grouped() for r in hist]
            return [r.to_grouped() for r in list(hist)[-int(limit):]]

    def snapshot(self) -> dict[str, dict[str, Any]]:
        """
        Return grouped snapshot for ALL types:
          { "TYPE": { "time": "...", "ver": N, "data": {...} }, ... }
        """
        with self._lock:
            return {t: rec.to_grouped() for t, rec in self._last_by_type.items()}

    def snapshot_since(self, versions_by_type: Dict[str, int]) -> Dict[str, Dict[str, Any]]:
        """
        Incremental snapshot based on per-type versions provided by the client.
        Input example: {"ATTITUDE": 120, "AHRS": 10}
        Output: only types whose last ver > provided one.
        If a type is not present in input, it is considered unseen => included if present in store.
        """
        with self._lock:
            out: Dict[str, Dict[str, Any]] = {}
            for t, rec in self._last_by_type.items():
                seen_ver = versions_by_type.get(t, 0)
                if rec.ver > seen_ver:
                    out[t] = rec.to_grouped()
            return out

    # -------- Maintenance --------

    def clear(self) -> None:
        """Drop all data and reset per-type versions."""
        with self._lock:
            self._versions_by_type.clear()
            self._last_by_type.clear()
            self._types_seen.clear()
            if self._keep_history:
                self._history.clear()

    def reset_type(self, msg_type: str) -> None:
        """Drop data for a single TYPE and reset its version."""
        with self._lock:
            self._versions_by_type.pop(msg_type, None)
            self._last_by_type.pop(msg_type, None)
            self._types_seen.discard(msg_type)
            if self._keep_history and msg_type in self._history:
                self._history.pop(msg_type, None)

    def reconfigure_history(self, history_size: int) -> None:
        """
        Change history size; affects future appends.
        Existing deques are rewrapped to new maxlen.
        """
        if history_size < 1:
            history_size = 1

        with self._lock:
            self._history_size = int(history_size)
            if not self._keep_history:
                return
            # Rewrap existing deques with the new maxlen
            new_hist: Dict[str, deque[MsgRecord]] = {}
            for t, dq in self._history.items():
                ndq = deque(dq, maxlen=self._history_size)
                new_hist[t] = ndq
            self._history = new_hist
