
from app.deps import get_joystick
from fastapi import APIRouter, Depends
from core.services.joystick import Joystick


router = APIRouter()


@router.post("/joystick-start")
def start_joystick(joystick: Joystick = Depends(get_joystick)):
    joystick.start_joystick()
    return {"joystick_run": joystick.running}


@router.post("/joystick-stop")
def stop_joystick(joystick: Joystick = Depends(get_joystick)):
    joystick.stop_joystick()
    return {"joystick_run": joystick.running}


@router.post("/joystick-start-rc")
def stop_joystick(joystick: Joystick = Depends(get_joystick)):
    joystick.start_rc()
    return {"joystick_rc_status": joystick.start_rc_flag}


@router.post("/joystick-stop-rc")
def stop_joystick(joystick: Joystick = Depends(get_joystick)):
    joystick.stop_rc()
    return {"joystick_rc_status": joystick.start_rc_flag}


@router.get("/joystick")
def joystick(joystick: Joystick = Depends(get_joystick)):
    data = joystick.rc_mapping_dict
    return data


@router.get("/joystick-name")
def joystick(joystick: Joystick = Depends(get_joystick)):
    return {"joystick_name": joystick.joystick_name}
