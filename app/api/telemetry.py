import time
from functools import wraps

from fastapi import APIRouter, Depends, HTTPException

from app.deps import get_mav, check_error
from app.schemas.telemetry import Telemetry
from core.services.mavlink_connection import MavlinkConnection

router = APIRouter()


@router.get("/all-telemetry")
@check_error
def all_telemetry(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.get_all_telemetry()
    return snap

@router.get("/all-types")
@check_error
def all_types(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.get_all_types()
    return snap

@router.get("/message/{msg_id}")
@check_error
def get_message(msg_id: str, mav: MavlinkConnection = Depends(get_mav)):
    res = mav.get_message(msg_id)
    return res

@router.get("/message-history/{msg_id}")
@check_error
def get_message_history(msg_id: str, mav: MavlinkConnection = Depends(get_mav)):
    res = mav.get_message_history(msg_id)
    return res

@router.get("/frontend-telemetry")
@check_error
def get_frontend_telemetry(mav: MavlinkConnection = Depends(get_mav)):
    return mav.get_frontend_telemetry()

