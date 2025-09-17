import json

from pymavlink import mavutil

FRAME_NAME_TO_ENUM = {
    "GLOBAL": mavutil.mavlink.MAV_FRAME_GLOBAL,
    "GLOBAL_RELATIVE_ALT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    "GLOBAL_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    "GLOBAL_RELATIVE_ALT_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
}
FRAME_ENUM_TO_NAME = {v: k for k, v in FRAME_NAME_TO_ENUM.items()}

CMD_NAME_TO_ENUM = {
    "NAV_WAYPOINT": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    "NAV_TAKEOFF": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    "NAV_LAND": mavutil.mavlink.MAV_CMD_NAV_LAND,
    "NAV_LOITER_TIME": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    "RETURN_TO_LAUNCH": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    "DO_CHANGE_SPEED": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
}
CMD_ENUM_TO_NAME = {v: k for k, v in CMD_NAME_TO_ENUM.items()}

def _frame_to_enum(frame):
    if isinstance(frame, int): return frame
    return FRAME_NAME_TO_ENUM.get(frame, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

def _cmd_to_enum(cmd):
    if isinstance(cmd, int): return cmd
    return CMD_NAME_TO_ENUM[cmd]


def dict_to_mission_item_int_fields(d: dict):
    """Build fields for mission_item_int_send from dict."""
    frame = _frame_to_enum(d.get("frame"))
    command = _cmd_to_enum(d.get("command"))
    p1, p2, p3, p4 = (d.get("params") or [0, 0, 0, 0])[:4]
    lat = float(d.get("lat", 0.0))
    lon = float(d.get("lon", 0.0))
    alt = float(d.get("alt", 0.0))
    x = int(round(lat * 1e7))
    y = int(round(lon * 1e7))
    z = alt
    current = int(d.get("current", 0))
    autocontinue = int(d.get("autocontinue", 1))
    return frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z

def mission_item_msg_to_dict(msg) -> dict:
    """Convert MISSION_ITEM_INT or MISSION_ITEM to dict."""
    frame = getattr(msg, "frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
    command = getattr(msg, "command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
    p1, p2, p3, p4 = msg.param1, msg.param2, msg.param3, msg.param4

    # INT vs non-INT
    if msg.get_type() == "MISSION_ITEM_INT":
        lat = msg.x / 1e7
        lon = msg.y / 1e7
    else:
        lat = msg.x
        lon = msg.y
    alt = msg.z

    return {
        "seq": int(getattr(msg, "seq", -1)),
        "frame": FRAME_ENUM_TO_NAME.get(frame, frame),
        "command": CMD_ENUM_TO_NAME.get(command, command),
        "lat": float(lat),
        "lon": float(lon),
        "alt": float(alt),
        "params": [float(p1), float(p2), float(p3), float(p4)],
        "current": int(getattr(msg, "current", 0)),
        "autocontinue": int(getattr(msg, "autocontinue", 1)),
    }
    
def save_mission_json(path: str, items: list[dict]):
    """Save mission dict list to JSON file."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)
    print(f"Saved mission to {path}")
    
def load_mission_json(path: str) -> list[dict]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError("Mission JSON must be a list of waypoints (dicts)")
    return data