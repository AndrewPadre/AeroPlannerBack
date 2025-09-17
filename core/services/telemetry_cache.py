import time
import math
from pymavlink import mavutil


# TelemetryCache produces a snapshot dict with the following keys:
#   roll, pitch, yaw
#   throttle, groundspeed, airspeed
#   bat_voltage, bat_amp, bat_cell_voltage
#   wind, heading_wind
#   lat, lon, alt_baro, alt_gps
#   dist_to_home, waypoint_number, dist_to_waypoint
#   gps_yaw, yaw_to_waypoint
#
# Each snapshot value is a tuple: ('new' | 'old', value)
#   - 'new' => value was updated during the current read cycle
#   - 'old' => value is taken from the previous cycle (not updated this time)

def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    # Great-circle distance in meters
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.asin(math.sqrt(a))

class TelemetryCache:
    """Keeps two dicts: 'new' (values updated this read cycle) and 'old' (previous cycle snapshot).
       Snapshot merges them, prioritizing 'new', labels entries as ('new'|'old', value),
       then rolls 'old' forward to the merged state and clears 'new'."""
    def __init__(self):
        # Per-cycle buffers
        self.new: dict[str, object] = {}  # values updated since last snapshot()
        self.old: dict[str, object] = {}  # previous merged snapshot

        # Helpers for derived fields
        self.home: tuple[float, float] | None = None

    # -------- internal helpers --------
    def _set_new(self, key: str, value: object) -> None:
        # Write into current-cycle buffer
        self.new[key] = value

    def _latest(self, key: str) -> object | None:
        # Read most-recent value: prefer 'new', fall back to 'old'
        if key in self.new:
            return self.new[key]
        return self.old.get(key)

    # -------- message ingestion --------
    def update_from_msg(self, msg) -> None:
        t = msg.get_type()

        if t == "ATTITUDE":
            # radians -> degrees
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = (math.degrees(msg.yaw) + 360.0) % 360.0
            self._set_new("roll", roll)
            self._set_new("pitch", pitch)
            self._set_new("yaw", yaw)
        elif t == "HEARTBEAT":
            self._set_new("mode", mavutil.mode_string_v10(msg))

        elif t == "VFR_HUD":
            if hasattr(msg, "groundspeed"):
                self._set_new("groundspeed", float(msg.groundspeed))
            if hasattr(msg, "airspeed"):
                self._set_new("airspeed", float(msg.airspeed))
            if hasattr(msg, "throttle"):
                self._set_new("throttle", float(msg.throttle))

        elif t == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt_gps = msg.alt / 1000.0  # mm -> m
            gps_yaw = msg.hdg
            self._set_new("lat", lat)
            self._set_new("lon", lon)
            self._set_new("alt_gps", alt_gps)
            self._set_new("gps_yaw", gps_yaw)
            if self.home is not None:
                d = _haversine_m(lat, lon, self.home[0], self.home[1])
                self._set_new("dist_to_home", d)

        elif t == "VFR_HUD":
            if hasattr(msg, "") and msg.altitude_amsl is not None:
                self._set_new("alt_baro", float(msg.altitude_amsl))

        elif t == "HOME_POSITION":
            self.home = (msg.latitude / 1e7, msg.longitude / 1e7)
            # If we already know a position (from new or old), compute distance immediately
            lat = self._latest("lat")
            lon = self._latest("lon")
            if lat is not None and lon is not None:
                d = _haversine_m(lat, lon, self.home[0], self.home[1])
                self._set_new("dist_to_home", d)

        elif t == "SYS_STATUS":
            vb_mv = getattr(msg, "voltage_battery", None)
            ib_ca = getattr(msg, "current_battery", None)
            if vb_mv not in (None, -1):
                self._set_new("bat_voltage", vb_mv / 1000.0)
            if ib_ca not in (None, -1):
                self._set_new("bat_amp", ib_ca / 100.0)

        elif t == "BATTERY_STATUS":
            volts = [v/1000.0 for v in getattr(msg, "voltages", []) if v != 65535 and v > 0]
            if volts:
                self._set_new("bat_cell_voltage", volts)

        elif t in ("WIND", "WIND_COV"):
            if t == "WIND":
                if hasattr(msg, "speed"):
                    self._set_new("wind", float(msg.speed))
                if hasattr(msg, "direction"):
                    self._set_new("heading_wind", float(msg.direction))
            else:
                if hasattr(msg, "wind_speed"):
                    self._set_new("wind", float(msg.wind_speed))
                if hasattr(msg, "wind_dir"):
                    self._set_new("heading_wind", float(msg.wind_dir))

        elif t == "MISSION_CURRENT":
            if hasattr(msg, "seq"):
                self._set_new("waypoint_number", int(msg.seq))

        elif t == "NAV_CONTROLLER_OUTPUT":
            if hasattr(msg, "wp_dist"):
                self._set_new("dist_to_waypoint", float(msg.wp_dist))
            if hasattr(msg, "target_bearing"):
                target_bearing = float(msg.target_bearing)
                yaw = self._latest("yaw")
                if yaw is not None:
                    d = (target_bearing - yaw + 540.0) % 360.0 - 180.0
                    self._set_new("yaw_to_waypoint", d)

        elif t == "GPS_RAW_INT":
            if hasattr(msg, "yaw") and msg.yaw not in (None, 65535):
                self._set_new("gps_raw_yaw", float(msg.yaw) * 0.01)
            if hasattr(msg, "lat"):
                self._set_new("raw_lat", float(msg.lat))
            if hasattr(msg, "lon"):
                self._set_new("raw_lon", float(msg.lon))
            

    # -------- snapshot & roll-forward --------
    def snapshot(self, include_all: bool = False, all_keys: list[str] | None = None) -> dict[str, tuple[str, object]]:
        """
        Merge 'new' over 'old' and return labeled dict:
            keys from 'new' -> ('new', value)
            remaining keys from 'old' -> ('old', value)
        Then commit: old = merged; new = {}.
        If include_all=True and all_keys provided, also include missing keys as ('old', None).
        """
        merged: dict[str, object] = {**self.old, **self.new}  # 'new' has priority
        out: dict[str, tuple[str, object]] = {}

        new_keys = set(self.new.keys())
        # Emit keys present in the merged state
        for k, v in merged.items():
            out[k] = ("new", v) if k in new_keys else ("old", v)

        # Optionally include a stable set of keys even if unseen yet
        if include_all and all_keys:
            for k in all_keys:
                if k not in out:
                    out[k] = ("old", None)

        # Roll forward: keep merged as previous state; clear current-cycle buffer
        self.old = merged
        self.new.clear()
        return out
