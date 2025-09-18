
from app.deps import get_joystick
from fastapi import APIRouter, Depends
from core.services.joystick import Joystick


router = APIRouter()


@router.get("/joystick")
def joystick(joystick: Joystick = Depends(get_joystick)):
    data = joystick.rc_mapping_dict
    return data

