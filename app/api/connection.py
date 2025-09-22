from fastapi import APIRouter, Depends
from app.deps import get_mav
from core.services.mavlink_connection import MavlinkConnection
from core.services.telemetry_service import TelemetryService


router = APIRouter()


@router.post("/connect")
def connect_async(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    mav.connect_to_uav(timeout=20)
    return {"connected": bool(mav.is_connected())}


@router.get("/status")
def status(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    return {"status": mav.state()} 


@router.post("/disconnect")
def disconnect(mav: MavlinkConnection=Depends(get_mav)) -> dict:
    ok = mav.disconnect_from_uav()
    return {"disconnected": bool(ok), "connected": mav.connect_status_flag}