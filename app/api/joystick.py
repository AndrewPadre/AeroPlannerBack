
from app.deps import get_mav, check_error
from fastapi import APIRouter, Depends
from core.services.mavlink_connection import MavlinkConnection


router = APIRouter()


@router.post("/connect")
@check_error
def connect(mav: MavlinkConnection = Depends(get_mav)):
    res = mav.connect_remote()
    return {"connected": res}


@router.post("/start")
@check_error
def start_rc_override(mav: MavlinkConnection = Depends(get_mav)):
    res = mav.start_rc_override()
    return {"joystick_status": res}

@router.post("/stop")
@check_error
def stop_rc_override(mav: MavlinkConnection = Depends(get_mav)):
    res = mav.stop_rc_override()
    return {"joystick_status": res}

@router.post("/disconnect")
@check_error
def disconnect(mav: MavlinkConnection = Depends(get_mav)):
    res = mav.disconnect_remote()
    return {}

@router.get("/rc-values")
@check_error
def rc_values(mav: MavlinkConnection = Depends(get_mav)):
    return mav.remote_rc_values
