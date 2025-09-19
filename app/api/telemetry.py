import time
from fastapi import APIRouter, Depends
from app.deps import get_mav
from app.schemas.telemetry import Telemetry
from core.services.mavlink_connection import MavlinkConnection

router = APIRouter()

@router.get("/telemetry")
def telemetry(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.read_data_from_uav() or {}
    flat = {k: v for k, (_, v) in snap.items()}
    return flat


@router.get("/telemetry-store")
def telemetry_store(mav: MavlinkConnection = Depends(get_mav)):
    snap = mav.telemetry_store()
    return snap

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

@router.post("/change-altitude/{altitude}")
def change_altitude(altitude: int, mav: MavlinkConnection = Depends(get_mav)):
    return mav.change_altitude(altitude)


# Temporary
flight_modes = ["STABILIZE", "ACRO", "FBWA", "FBWB", "MANUAL"]

@router.post("/change-mode")
def change_mode(mav: MavlinkConnection=Depends(get_mav)):
    for mode in flight_modes:
        print(mav.change_flight_mode(mode))
    return "success"