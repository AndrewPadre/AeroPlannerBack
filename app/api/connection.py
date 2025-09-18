from fastapi import APIRouter, Depends
from app.deps import get_mav
from core.services.mavlink_connection import MavlinkConnection
from core.services.telemetry_service import TelemetryService


router = APIRouter()

@router.post("/connect")
def connect(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    ok = mav.connect_to_uav()
    if ok:
        TelemetryService(mav)
    return {"connected": bool(ok)}

@router.post("/connect-async")
def connect_async(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    mav.connect_async(timeout=20)
    return {"connected": bool(mav.is_connected())}


@router.get("/status")
def status(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    return {"connected": mav.connect_status_flag}  # TODO replace by heartbeat

@router.get("/status-async")
def status(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    return {"status": mav.state()} 

@router.post("/disconnect")
def disconnect(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    ok = mav.disconnect_from_uav()
    return {"disconnected": bool(ok), "connected": mav.connect_status_flag}


@router.post("/disconnect-async")
def disconnect(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    ok = mav.disconnect()
    return {"disconnected": bool(ok), "connected": mav.connect_status_flag}