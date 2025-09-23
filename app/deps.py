import threading

from functools import wraps
from fastapi import HTTPException

from app.settings import settings
from core.services.gimbal_view_pro import GimbalViewPro
from core.services.mavlink_connection import MavlinkConnection

_mav = MavlinkConnection(settings.MAVLINK_URL)

def check_error(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        if isinstance(result, dict) and "error" in result:
            raise HTTPException(status_code=401, detail=result["error"])
        return result
    return wrapper


"""-----GIMBAL VIEW PRO-----"""
_gimbal_view_pro = GimbalViewPro()
# _gimbal_view_pro.start()


def get_mav() -> MavlinkConnection:
    return _mav

def get_gimbal_view_pro() -> GimbalViewPro:
    return _gimbal_view_pro