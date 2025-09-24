import threading

from functools import wraps
from fastapi import HTTPException

from app.settings import settings
from core.services.gimbal_view_pro import GimbalViewPro
from core.services.mavlink_connection import MavlinkConnection

_mav = MavlinkConnection(settings.MAVLINK_URL)

def _to_camel(s: str) -> str:
    parts = s.split('_')
    return parts[0] + ''.join(p.capitalize() for p in parts[1:])

def _camelize(obj):
    # Recursively convert dict keys to camelCase
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            kk = _to_camel(k) if isinstance(k, str) else k
            out[kk] = _camelize(v)
        return out
    if isinstance(obj, list):
        return [_camelize(x) for x in obj]
    return obj

def camelize_response(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        res = func(*args, **kwargs)
        # Only transform JSON-like payloads
        if isinstance(res, (dict, list)):
            return _camelize(res)
        return res
    return wrapper

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