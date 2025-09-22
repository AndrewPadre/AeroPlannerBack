from fastapi import APIRouter, Depends
from app.deps import get_mav
from core.services.mavlink_connection import MavlinkConnection
from core.services.telemetry_service import TelemetryService


router = APIRouter()


@router.post("/arm")
def arm(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    res = mav.arm()
    return {"armed": res}


@router.post("/disarm")
def disarm(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    res = mav.disarm()
    return {"disarmed": res}


@router.post("/change-mode/{mode}")
def change_mode(mode: str, mav: MavlinkConnection=Depends(get_mav)) -> dict:
    res = mav.change_flight_mode(mode)
    return {"mode_changed": res}

@router.post("/change-altitude/{altitude}")
def change_altitude(altitude: int, mav: MavlinkConnection = Depends(get_mav)):
    return mav.change_altitude(altitude)

