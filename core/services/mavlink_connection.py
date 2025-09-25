import sys
import time
import math
import functools
import threading, time, enum
from collections import deque
import random
from typing import Any
from queue import Queue


from pymavlink import mavutil
from geopy.distance import geodesic

from core.services.telemetry_cache import TelemetryCache
from core.services.mission_utils import dict_to_mission_item_int_fields, mission_item_msg_to_dict, save_mission_json, load_mission_json
from core.services.telemetry_store import TelemetryStore
from core.services.joystick import RemoteController


# TODO 
# Go asynchronous
# Replace telemetry_cache.py
# Add logger
# Add type hints
# Add docstring
# Make check function call 1 time


class ConnState(str, enum.Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING   = "CONNECTING"
    CONNECTED    = "CONNECTED"
    RECONNECTING = "RECONNECTING"


class RemoteConnState(str, enum.Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED    = "CONNECTED"
    RECONNECTING = "RECONNECTING"
    # Non need for CONNECTING   



def requires_connection(func):
    """Decorator: ensures UAV is connected before running the function."""
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            if self.is_connected():
                return func(self, *args, **kwargs)
            else:
                print(f"Cannot execute '{func.__name__}': UAV not connected.")
                return {"error": "UAV is not connected"}
        except AttributeError:
            print(f"Cannot execute '{func.__name__}': 'is_connected()' not defined.")
            return {"error": "attribute error"}
    return wrapper


class MavlinkConnection:
    TIMEOUT = 3  # Standart timeout for mavlink functions
    LOST_SEC = 5  # Delay from last heartbeat to switch state to RECONNECTING
    CONNECT_TIMEOUT = 30  # How long it will take to 
    BATT_CELLS = 3

    def __init__(self, port, baudrate=None):
        self.port = port
        self.baudrate = baudrate
        self.master: Any = None
        self.connect_status_flag = False
        self.is_armed = False  # Ask
        self.curr_mode: str = "Manual"
        self._telemetry_cache = TelemetryCache()
        self._store = TelemetryStore()
        self.check_if_armed
        self._attitude_rate = 4  # default rate in hz
        self._global_int_pos_rate = 4  # default
        self._att_last_t = None          # last arrival timestamp (monotonic)
        self._last_command_ack: Any = None # Last ACK message(confirmartion)
        self._command_ack_lock = threading.Lock()
        
        # New
        self._state_lock = threading.Lock()
        self._connected_evt = threading.Event()  # TODO delete
        self._last_hb_ts: float = 0.0
        self._state = ConnState.DISCONNECTED
        self._stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        
        self.remote_controller = RemoteController()
        self._remote_run_event = threading.Event()  # event for starting loop that takes rc_values from remote
        self._remote_send_event = threading.Event()  # event that trigger sending rc_values to MAV
        self._remote_thread: threading.Thread = threading.Thread(target=self._remote_loop, daemon=True)
        self.remote_rc_values: dict = self.remote_controller.do_default_rc_mapping_dict()
        self._remote_state = RemoteConnState.DISCONNECTED
        
        self.is_in_air: bool = False
        self._airborne_since: float | None = None   # monotonic timestamp when went airborne
        self._time_in_air: float = 0.0  
        self._prev_latlon: tuple[float, float] | None = None
        self._prev_save_travel_time: float | None = None
        self._travel_dist_m: float = 0.0

        

    def state(self) -> ConnState:
        with self._state_lock: 
            return self._state
        

    def is_connected(self) -> bool:
        return self.state() == ConnState.CONNECTED

            
    def connect_to_uav(self, timeout: float = 8.0) -> None:
        """Connecting to mav for timeout"""
        with self._state_lock:
            if self._state in (ConnState.CONNECTING, ConnState.CONNECTED, ConnState.RECONNECTING):
                return
            self._state = ConnState.CONNECTING
            self._last_err = None
            self._connected_evt.clear()
        # self._connect_thread = threading.Thread(target=self._connect_worker, args=([timeout]), daemon=True)
        # self._connect_thread.start()
        print("Connecting")
        ok = self._try_connect_once(timeout)
        if ok:
            self._start_reader_and_watchdog()
    
    # TODO remove
    def wait_connected(self, timeout: float) -> bool:
        """Wait for the connect"""
        return self._connected_evt.wait(timeout=timeout)

        
    
    def disconnect_from_uav(self) -> bool:
        """Disconnect from the mav, close all service threads"""
        self._stop.set()
        try:
            if self._reader_thread and self._reader_thread.is_alive():
                self._reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.master is not None:
                try:
                    self.master.close()
                except Exception:
                    pass
            self.master = None
        finally:
            with self._state_lock:
                self._state = ConnState.DISCONNECTED
            self._connected_evt.clear()
            self._stop.clear()
        return True
    
    # ________________JOYSTICK___________________
    def connect_remote(self) -> bool:
        res = self.remote_controller.connect()
        print(res)
        if not res:
            self._remote_state = RemoteConnState.DISCONNECTED
            return False
        self._remote_state = RemoteConnState.CONNECTED
        self._remote_run_event.set()
        self._remote_thread.start()
        return res
    
    def disconnect_remote(self):
        self.remote_controller.disconnect()
        self._remote_state = RemoteConnState.DISCONNECTED
        self._remote_run_event.clear()
        return self._remote_state
        
    
    @requires_connection
    def start_rc_override(self):
        self._remote_send_event.set()
        return self._remote_state

    @requires_connection 
    def stop_rc_override(self):
        self._remote_send_event.clear()
        return self._remote_state
    
    def _remote_loop(self):
        while self._remote_run_event.is_set():
            # Sleep if joystick is not connected
            if not self.remote_controller.is_joystick_connected():
                self._remote_state = RemoteConnState.RECONNECTING  # TODO make timeout on reconnecting
                time.sleep(0.5)
            self._remote_state = RemoteConnState.CONNECTED
            self._remote_rc_values = self.remote_controller.get_snapshot()
            #  Sending rc to mav
            print(self._remote_send_event.is_set())
            if self._remote_send_event.is_set():
                self._send_rc(self._remote_rc_values)
            time.sleep(0.1)
            
    @requires_connection    
    def _send_rc(self, rc_values: dict[str, dict[int, dict]]):
        rc_channel_values = [65535 for _ in range(18)]
        for key, value in rc_values["RC"].items():
            rc_channel_values[key - 1] = value.get("pwm_value", 1500)

        self.master.mav.rc_channels_override_send(
            self.master.target_system,                
            self.master.target_component,            
            *rc_channel_values)
        
        
                    
    def _try_connect_once(self, timeout: float) -> bool:
        """
        Try to establish a single connection attempt and wait for heartbeat.
        Returns True only if heartbeat is received within the given timeout.
        """

        try:
            # Create MAVLink connection (serial with baudrate or UDP/TCP)
            if self.baudrate is not None:
                self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate)
            else:
                self.master = mavutil.mavlink_connection(self.port)

            # Wait for heartbeat (blocks until timeout)
            print("Waiting for heartbeat")
            hb = self.master.wait_heartbeat(timeout=timeout)

            if hb is None:
                print("No heartbeat")
                # No heartbeat received -> treat as failed connection
                self._last_err = f"No heartbeat received within {timeout} seconds"
                self._connected_evt.clear()
                with self._state_lock:
                    self._state = ConnState.DISCONNECTED
                return False
            print("Heartbeat received")

            # Heartbeat received -> connection is considered established
            self._last_hb_ts = time.time()
            with self._state_lock:
                self._state = ConnState.CONNECTED
            self._connected_evt.set()
            return True

        except Exception as e:
            # Any error during connection attempt -> failed
            self._last_err = str(e)
            self._connected_evt.clear()
            with self._state_lock:
                self._state = ConnState.DISCONNECTED
            return False
        
    def get_time_in_air_minutes(self) -> float:
        if self._airborne_since and self.is_in_air:
            return time.time() - self._airborne_since / 60
        return 0.0


    def _check_if_in_air(self, groundspeed, airspeed):
        prev_in_air = self.is_in_air
        if groundspeed:
            if groundspeed > 15:
                self.is_in_air = True
            elif groundspeed < 10:
                self.is_in_air = False
        else:
            if airspeed > 15:
                self.is_in_air = True 
            elif airspeed < 10:
                self.is_in_air = False 
        
        now = time.time()

        # Rising edge: just went airborne -> start timer
        if self.is_in_air and not prev_in_air:
            self._airborne_since = now

        # Falling edge: landed -> accumulate and stop timer
        if (not self.is_in_air) and prev_in_air:
            if self._airborne_since is not None:
                self._airborne_since = None

    
    # Loop
    def _start_reader_and_watchdog(self):
        def reader():
            while not self._stop.is_set():
                try:
                    self.telemetry_store()  # Update telemetry store, read data from the UAV
                    # TODO move to method
                    groundspeed = self._store.get_last("VFR_HUD").get("data", {}).get("groundspeed", 0)
                    airspeed = self._store.get_last("VFR_HUD").get("data", {}).get("airspeed", 0)
                    spd = None
                    if self.is_gps_fix():
                        spd = groundspeed
                    else:
                        spd = airspeed
                        
                    self._check_if_in_air(groundspeed, airspeed)
                    lat = self._store.get_last("GLOBAL_POSITION_INT").get("data", {}).get("lat", 0)
                    lon = self._store.get_last("GLOBAL_POSITION_INT").get("data", {}).get("lon", 0)
                    self._accum_travel_by_pos(lat, lon, spd)
                    
                        
                except Exception as e:
                    time.sleep(0.2)
                    print("Exception occured", e)

        def watchdog():
            while not self._stop.is_set():
                if self.is_connected() and (time.time() - self._last_hb_ts > self.LOST_SEC):
                    print("Reconnecting")
                    with self._state_lock:
                        self._state = ConnState.RECONNECTING
                    # self.disconnect()
                    # self.connect_async(timeout=5.0, auto_reconnect=True)
                elif not self.is_connected() and (time.time() - self._last_hb_ts < self.LOST_SEC):
                    with self._state_lock:
                        self._state = ConnState.CONNECTED
                    self._connected_evt.set()
                time.sleep(0.5)
        # TODO consider deleting one thread
        self._reader_thread = threading.Thread(target=reader, daemon=True)
        self._reader_thread.start()
        threading.Thread(target=watchdog, daemon=True).start()

    @requires_connection 
    def get_all_telemetry(self) -> dict:
        """Provides all last telemetry messages"""
        snap = self._store.snapshot()
        return snap
    
    @requires_connection 
    def get_all_types(self) -> list:
        return self._store.get_types()
    
    @requires_connection 
    def get_message(self, msg_id) -> dict:
        return self._store.get_last(msg_id)
    
    @requires_connection 
    def get_message_history(self, msg_id) -> list:
        return self._store.get_history(msg_id, 50)  # TODO remove hardcode
    
    @requires_connection
    def get_frontend_telemetry(self) -> dict:
        gps = self._store.get_last("GPS_RAW_INT").get("data", {})
        sys_status = self._store.get_last("SYS_STATUS").get("data", {})
        battery = self._store.get_last("BATTERY_STATUS").get("data", {})
        att = self._store.get_last("ATTITUDE").get("data", {})
        vfr = self._store.get_last("VFR_HUD").get("data", {})
        home = self._store.get_last("HOME_POSITION").get("data", {})
        global_pos = self._store.get_last("GLOBAL_POSITION_INT").get("data", {})

        sats = gps.get("satellites_visible")
        hdop = gps.get("eph", 0) / 100.0 if gps.get("eph") else None
        vbat = sys_status.get("voltage_battery", 0) / 1000.0
        vcell = vbat / self.BATT_CELLS  # TODO consider replacement
        current = sys_status.get("current_battery")
        used_mah = battery.get("current_consumed")
        roll = round(math.degrees(att.get("roll", 0)), 2)
        pitch = round(math.degrees(att.get("pitch", 0)), 2)
        yaw = round(math.degrees(att.get("yaw", 0)))
        airspeed = vfr.get("airspeed")
        groundspeed = vfr.get("groundspeed")


        # distance to home (if both positions exist)
        dist_home = 0
        time_to_home = 0
        print("global_pos", global_pos)
        print("home", home)
        if global_pos and home:
            lat_now = global_pos["lat"] * 1e-7
            lon_now = global_pos["lon"] * 1e-7
            lat_home = home["latitude"] * 1e-7
            lon_home = home["longitude"] * 1e-7

            dist_home = geodesic((lat_now, lon_now), (lat_home, lon_home)).meters

            speed = groundspeed or airspeed
            if speed and speed > 0:
                time_to_home = dist_home / speed  
        
        return {
            "gps_sats": sats,
            "hdop": hdop,
            "vbat": vbat,
            "vcell": vcell,
            "used_mah": used_mah,
            "current": current,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "airspeed": airspeed,
            "groundspeed": groundspeed,
            "dist_to_home": dist_home,
            "time_in_air": self.get_time_in_air_minutes(), 
            "time_to_home": time_to_home,
            "mode": self.curr_mode,
            "travel_dist": self._travel_dist_m
        }
        
    

    def _accum_travel_by_pos(self, lat, lon, spd):
        """Accumulate travel distance using positions (GLOBAL_POSITION_INT)."""
        if lat is None or lon is None:
            return
        lat = float(lat) * 1e-7
        lon = float(lon) * 1e-7
        print("dist ", self._travel_dist_m)
        if self.is_gps_fix():
            if self._prev_latlon is not None:
                d = geodesic(self._prev_latlon, (lat, lon)).meters
                print("d", d)
                # Ignore GPS jitter under 0.5 m and add only when in air
                if self.is_in_air:
                    self._travel_dist_m += d
            self._prev_latlon = (lat, lon)
            self._prev_save_travel_time = None
        else:
            if self._prev_save_travel_time is not None:
                estimate = time.time() - self._prev_save_travel_time
                d = estimate * spd
                self._travel_dist_m += d
            self._prev_save_travel_time = time.time()
            self._prev_latlon = None

    
    def is_gps_fix(self) -> bool:
        return self._store.get_last("GPS_RAW_INT").get("data", {}).get("fix_type", 0) >= 2

    
    

    @requires_connection
    def _set_rate(self, rate_hz: float, msg_id: int):
        interval_us = 0 if rate_hz <= 0 else int(1_000_000 / float(rate_hz))

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),          # param1: message id
            float(interval_us),     # param2: interval in usec (0=disable, -1=default)
            0, 0, 0, 0, 0
        )

        deadline = time.time() + self.TIMEOUT
        while time.time() < deadline:
            ack = self._last_command_ack
            if not ack:
                continue
            if getattr(ack, "command", None) == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                ok = ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                if not ok:
                    print(f"SET_MESSAGE_INTERVAL rejected: result={ack.result}")
                return ok

        print("SET_MESSAGE_INTERVAL: no COMMAND_ACK")
        return False

    
    @requires_connection
    def set_global_int_pos_rate(self, rate_hz: float) -> bool:
        """Sets attitude(roll, pitch, yaw) rate in hz"""
        self._global_int_pos_rate = rate_hz
        msg_id = int(getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_GLOBAL_POSITION_INT"))
        return self._set_rate(rate_hz, msg_id)

    @requires_connection
    def set_attitude_rate(self, rate_hz: float) -> bool:
        """Sets attitude(roll, pitch, yaw) rate in hz"""
        self._attitude_rate = rate_hz
        msg_id = int(getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_ATTITUDE"))
        return self._set_rate(rate_hz, msg_id)


    # Deprecated
    @requires_connection
    def check_if_armed(self) -> bool:
        hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    
    @requires_connection
    def arm(self) -> bool:
        # sending arm via mavlink
        if not self.is_armed:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0)
        start_time = time.time()
        # Waiting for the arm
        while not self.is_armed and time.time() - start_time < self.TIMEOUT:
            time.sleep(0.5)

        # Setting is_armed field
        if self.is_armed:
            print("Armed")
            return True
        else:
            print("Cannot arm")
            return False
        
    @requires_connection
    def disarm(self) -> bool:
        # Sending disarm via mavlink
        if not self.is_armed:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0, 0, 0, 0, 0, 0, 0)
        start_time = time.time()
        # Waiting for the disarm
        while self.check_if_armed() and time.time() - start_time < self.TIMEOUT:
            time.sleep(0.5)
        # Setting is_armed field
        if self.is_armed:
            print("Disarmed")
            return True
        else:
            print("Cannot disarm")
            return False
        

    @requires_connection
    def change_flight_mode(self, flight_mode) -> bool:
        # Validate flight mode
        if flight_mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(flight_mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            return False
        # Get flight mode id
        mode_id = self.master.mode_mapping()[flight_mode]

        # Change flight mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        
        start_time = time.time()
        while self.curr_mode != flight_mode and time.time() - start_time < 5:
            print(self.curr_mode)
            time.sleep(0.)
        if self.curr_mode == flight_mode:
            print(f"Set flight mode to {flight_mode}")
            return True
        else:
            print(f"Can't change flight mode to {flight_mode}")
            return False


    @requires_connection
    def set_parameter(self, name, value) -> bool:
        # Get type of the value
        if isinstance(value, bool):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_UINT8
            numeric_value = 1.0 if value else 0.0
        elif isinstance(value, int):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            numeric_value = float(value)
        elif isinstance(value, float):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            numeric_value = float(value)
        else:
            raise TypeError("value must be bool, int, or float")

        # Send PARAM_SET
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("ascii"),
            numeric_value,
            param_type
        )

        start_time = time.time()
        while self.get_parameter(name) != value and time.time() - start_time < self.TIMEOUT:
            time.sleep(0.5)

        if self.get_parameter(name) == value:
            print(f"Param {name} successfully set to {value}")            
            return True
        else:
            print(f"Can't set param {name} to value {value}")
            return False


    @requires_connection
    def get_parameter(self, name: str) -> int|float:
        self.master.mav.param_request_read_send(
        self.master.target_system, self.master.target_component,
        name.encode("ascii"),  # Encode to bytes
        -1
        )

        message = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=self.TIMEOUT).to_dict()
        print(f"get_parameter: name: {message['param_id']}, value: {message['param_value']}")
        return message['param_value']   



    @requires_connection
    def upload_mission(self, items: list[dict], timeout: float = 8.0) -> bool:
        print(items)
        """Upload mission (MISSION_ITEM_INT) to vehicle. Returns True on success."""
        count = len(items)
        t_sys, t_comp = self.master.target_system, self.master.target_component
        mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        # Send mission count
        self.master.mav.mission_count_send(t_sys, t_comp, count, mtype)

        sent = 0
        deadline = time.time() + timeout
        while sent < count and time.time() < deadline:
            req = self.master.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                                         blocking=True, timeout=0.5)
            if not req:
                continue
            seq = int(req.seq)
            if seq < 0 or seq >= count:
                continue

            frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z = dict_to_mission_item_int_fields(items[seq])
            self.master.mav.mission_item_int_send(
                t_sys, t_comp, seq, frame, command, current, autocontinue,
                p1, p2, p3, p4, x, y, z, mtype
            )
            sent += 1

        # Expect MISSION_ACK
        ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=2.0)
        if ack and getattr(ack, "type", None) in (mavutil.mavlink.MAV_MISSION_ACCEPTED, 0):
            print(f"Mission upload: OK ({count} items)")
            return True

        print(f"Mission upload: FAILED (sent {sent}/{count})")
        return False


    # TODO Make first point - home
    @requires_connection
    def download_mission(self) -> list[dict]:
        t_sys, t_comp = self.master.target_system, self.master.target_component
        mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        # Get count of the mission items
        self.master.mav.mission_request_list_send(t_sys, t_comp, mtype)
        cnt_msg = self.master.recv_match(type="MISSION_COUNT", blocking=True, timeout=self.TIMEOUT)
        if not cnt_msg:
            print("Mission download: no MISSION_COUNT")
            return []

        count = int(cnt_msg.count)
        items = [None] * count

        # Request each mission item
        for seq in range(count):
            self.master.mav.mission_request_int_send(t_sys, t_comp, seq, mtype)
            msg = self.master.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"], blocking=True, timeout=self.TIMEOUT)
            if not msg:
                print(f"Mission download: timeout at seq {seq}")
                return [i for i in items if i is not None]
            items[seq] = mission_item_msg_to_dict(msg)

        # End of the mission request
        self.master.recv_match(type="MISSION_ACK", blocking=False)

        print(f"Mission download: OK ({count} items)")
        return items



    @requires_connection
    def change_airspeed(self, airspeed: float) -> bool:
        with self._command_ack_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                0,                # speed type: 0 = airspeed
                float(airspeed),  # target airspeed (m/s)
                -1,               # throttle: ignore
                0, 0, 0, 0
            )

            # TODO consider deleting while loop
            # Wait for the response
            start_time = time.time()
            while time.time() - start_time < self.TIMEOUT:
                ack = self._last_command_ack
                if not ack:
                    continue
                if getattr(ack, "command", None) == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
                    ok = ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                    print(f"Change airspeed ACK: {ack.result} ({'OK' if ok else 'FAIL'})")
                    return ok

            print("Change airspeed: no COMMAND_ACK received")
            return False


    @requires_connection
    def change_altitude(self, altitude: float) -> bool:
        with self._command_ack_lock:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
                0,
                float(altitude),  # param1: Altitude (m)
                int(frame),       # param2: Frame (MAV_FRAME)
                0, 0, 0, 0, 0     # param3..param7 unused
            )

            # TODO consider deleting while loop
            # time.sleep(0.02)
            # Wait for COMMAND_ACK
            start_time = time.time()
            while time.time() - start_time < self.TIMEOUT:
                ack = self._last_command_ack
                print(ack)
                if not ack:
                    continue
                if getattr(ack, "command", None) == mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE:
                    ok = ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                    print(f"Change altitude ACK: {ack.result} ({'OK' if ok else 'FAIL'})")
                    self._last_command_ack = None
                    return ok

            print("Change altitude: no COMMAND_ACK received")
            return False



    @requires_connection
    def send_fake_gps(self):
        pass
    

    def _check_attitude_rate(self):
        """Update EMA of ATTITUDE frequency. Ultra-lightweight."""
        now = time.perf_counter()  # monotonic
        if self._att_last_t is not None:
            dt = now - self._att_last_t
            if dt > 0:
                inst_hz = 1.0 / dt
                # First sample initializes EMA

                self._att_ema_hz = inst_hz if self._att_ema_hz is None else \
                                   (0.2 * inst_hz + 0.8 * self._att_ema_hz)
        self._att_last_t = now
        return self._att_ema_hz

    @requires_connection
    def read_data_from_uav(self, poll_window: float = 0.05) -> dict[str, tuple[str, object]]:
        # print(threading.current_thread())
        # self._lock.acquire()
        end = time.time() + poll_window
        # read for 0.05s or until get data
        while time.time() < end:
            msg = self.master.recv_match(blocking=True)
            if not msg:
                # tiny sleep to avoid busy loop
                # time.sleep(0.005)
                continue
            # if msg.get_type() == "ATTITUDE":
            #     if self._att_ema_hz < 18:
            #         print("send")
            #         self.set_attitude_rate(self._attitude_rate)

            if msg.get_type() == "BAD_DATA":
                print("Mavlink BAD DATA")
                continue
            print("msg", msg)
            self._telemetry_cache.update_from_msg(msg)
        # self._lock.release()
        return self._telemetry_cache.snapshot()

    @requires_connection
    def telemetry_store(self) -> dict[str, dict[str, object]]:
        end = time.time() + 1
        while time.time() < end:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if not msg:
                print("continue")
                # time.sleep(0.005)
                continue
            t:str = msg.get_type()
            if t == "HEARTBEAT":
                self._last_hb_ts = time.time()
                self.curr_mode = mavutil.mode_string_v10(msg)
                self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            elif t == "COMMAND_ACK":  # Write las command_ack message, for commands that requires command_ack
                self._last_command_ack = msg
            elif t.startswith("UNKNOWN"):
                continue
            
            d = msg.to_dict()
            self._store.ingest(t, d)

        return self._store.snapshot() 


    def do_mavlink_cmd(self):
        pass

    @requires_connection
    def send_rc(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 16:
            return
        rc_channel_values = [65535 for _ in range(16)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values)

        print(f"Channel {channel_id + 1}: {pwm} PWM")


if __name__ == '__main__':
    mavlink = MavlinkConnection('COM5', 115200)
    mavlink.connect_to_uav()
    mavlink.set_attitude_rate("ATT", 20)
    while True:
        data = mavlink.read_data_from_uav()
        print(data.get("roll"))
        time.sleep(0.05)
    # new_func(mavlink)
    # for i in range(100):
    #     print(mavlink.read_data_from_uav())
    #     time.sleep(0.1)