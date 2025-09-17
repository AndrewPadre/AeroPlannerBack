# Single telemetry DTO used across the app (HUD, map, panels).
# Comments in English.

from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class TelemetryDTO:
    # Attitude / heading
    roll: float = 0.0
    pitch: float = 0.0

    yaw: float = 0.0  
    gps_yaw: float = 0.0
    yaw_to_wp: float = 0.0
    gps_raw_yaw: float = 0.0 

    # Speeds / altitude (HUD expects spd & alt)
    spd: float = 0.0           # airspeed for HUD
    alt: float = 0.0           # preferred altitude for HUD (baro first)

    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_baro: Optional[float] = None
    alt_gps: Optional[float] = None
    groundspeed: Optional[float] = None
    airspeed: Optional[float] = None
    
    raw_lat: Optional[float] = None
    raw_lon: Optional[float] = None

    # Power/etc. (extend as needed)
    bat_voltage: Optional[float] = None
    mode: str = None

    @classmethod
    def from_snapshot(cls, s: Dict[str, Any]) -> "TelemetryDTO":
        """Build DTO from TelemetryService.snapshot() (flat dict)."""
        # Prefer baro alt, fallback to GPS
        alt_pref = s.get("alt_baro", None)
        if alt_pref is None:
            alt_pref = s.get("alt_gps", 0.0)
        # Prefer airspeed for HUD 'spd'
        spd_pref = s.get("airspeed", 0.0)
        return cls(
            roll=float(s.get("roll", 0.0) or 0.0),
            pitch=float(s.get("pitch", 0.0) or 0.0),
            spd=float(spd_pref or 0.0),
            alt=float(alt_pref or 0.0),
            lat=s.get("lat", 0),
            lon=s.get("lon", 0),
            alt_baro=s.get("alt_baro", 0),
            alt_gps=s.get("alt_gps", 0),
            groundspeed=s.get("groundspeed", 0),
            airspeed=s.get("airspeed", 0),
            bat_voltage=s.get("bat_voltage", 0),
            yaw=s.get("yaw", 0),
            gps_yaw=s.get("gps_yaw", 0),
            yaw_to_wp=s.get("yaw_to_waypoint", 0),
            gps_raw_yaw=s.get("gps_raw_yaw", 0) * 0.01,
            raw_lat=s.get("raw_lat", 0) / 10e6,
            raw_lon=s.get("raw_lon", 0) / 10e6,
            mode=s.get("mode", "Manual"),
        )

