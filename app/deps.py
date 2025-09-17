from app.settings import settings
from core.services.mavlink_connection import MavlinkConnection

_mav = MavlinkConnection(settings.MAVLINK_URL)

def get_mav() -> MavlinkConnection:
    return _mav
