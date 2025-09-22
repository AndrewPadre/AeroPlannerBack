import time
from fastapi import APIRouter, Depends
from app.deps import get_mav
from app.schemas.telemetry import Telemetry
from core.services.mavlink_connection import MavlinkConnection

router = APIRouter()


@router.get("/all-telemetry")
def all_telemetry(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.get_all_telemetry()
    print(snap)
    return snap

@router.get("/all-types")
def all_types(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.get_all_types()
    print(snap)
    return snap

@router.get("/message/{msg_id}")
def get_message(msg_id: str, mav: MavlinkConnection = Depends(get_mav)):
    res = mav.get_message(msg_id)
    return res

@router.get("/message-history/{msg_id}")
def get_message_history(msg_id: str, mav: MavlinkConnection = Depends(get_mav)):
    res = mav.get_message_history(msg_id)
    return res


