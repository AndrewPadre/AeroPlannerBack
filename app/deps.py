from app.settings import settings
from core.services.joystick import Joystick
from core.services.gimbal_view_pro import GimbalViewPro
from core.services.mavlink_connection import MavlinkConnection

_mav = MavlinkConnection(settings.MAVLINK_URL)

_joystick = Joystick()
_joystick.start()

_gimbal_view_pro = GimbalViewPro()
_gimbal_view_pro.start()


def get_mav() -> MavlinkConnection:
    return _mav

def get_joystick() -> Joystick:
    return _joystick

def get_gimbal_view_pro() -> GimbalViewPro:
    return _gimbal_view_pro