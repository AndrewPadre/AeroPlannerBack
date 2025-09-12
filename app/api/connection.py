from fastapi import APIRouter, Depends
from app.deps import get_mav
from core.services.mavlink_connection import MavlinkConnection

router = APIRouter()

@router.post("/connect")
def connect(mav: MavlinkConnection= Depends(get_mav)) -> dict:
    ok = mav.connect_to_uav()
    return {"connected": bool(ok)}

@router.get("/status")
def status(mav: MavlinkConnection = Depends(get_mav)) -> dict:
    return {"connected": mav.connect_status_flag}  # TODO replace by heartbeat

@router.post("/disconnect")
def disconnect(mav: MavlinkConnection = Depends(get_mav)) -> dict:
    ok = mav.disconnect_from_uav()
    return {"disconnected": bool(ok), "connected": mav.connect_status_flag}
