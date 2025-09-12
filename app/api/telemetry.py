from fastapi import APIRouter, Depends
from app.deps import get_mav
from app.schemas.telemetry import Telemetry
from core.services.mavlink_connection import MavlinkConnection
from core.dto.telemetry_dto import TelemetryDTO

router = APIRouter()

@router.get("/telemetry")
def telemetry(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.read_data_from_uav()
    print("snap", snap)
    flat = {k: v for k, (_, v) in snap.items()}
    return flat
