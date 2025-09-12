import sys
import time
import math
import functools


from pymavlink import mavutil

from core.services.telemetry_cache import TelemetryCache
from core.services.mission_utils import dict_to_mission_item_int_fields, mission_item_msg_to_dict, save_mission_json, load_mission_json

import threading

# TODO 
# Go asynchronous
# Replace telemetry_cache.py
# Add logger
# Add type hints
# Add docstring
# Make check function call 1 time


def requires_connection(func):  # decorator for check that uav is connected
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if getattr(self, 'connect_status_flag', False):
            return func(self, *args, **kwargs)
        else:
            print(f"Cannot execute '{func.__name__}': UAV not connected.")
            return None
    return wrapper

from collections import deque

class MavlinkConnection:
    TIMEOUT = 3
    def __init__(self, port, baudrate=None):
        self.port = port
        self.baudrate = baudrate
        self.master = None
        self.connect_status_flag = False
        self.is_armed = False  # Ask
        # TODO self.curr_mode ask
        self._telemetry_cache = TelemetryCache()
        self._lock = threading.Lock()
        self._attitude_rate = 4  # default rate in hz
        self._global_int_pos_rate = 4  # default
        self._att_last_t = None          # last arrival timestamp (monotonic)
        self._att_ema_hz = 0          # EWMA of instantaneous frequency
        self._att_win = deque(maxlen=64) # rolling window of timestamps
        self._att_last_report = 0.0      # last time we updated window stats
        self._att_report_interval = 1.0  # seconds between window recompute
        self._att_window_hz = 0.0        # last computed window frequency
        self._att_alpha = 0.2            # EWMA smoothing factor (0..1)
        self._att_debug = True  
        self.inc = 0


    def connect_to_uav(self):
        """
        Function for connecting UAV to ground station
        :return: bool status that UAV is connected to GS
        """
        while not self.connect_status_flag:
            try:
                if self.baudrate is not None:
                    self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate)
                else:
                    self.master = mavutil.mavlink_connection(self.port)
                print(type(self.master))

                print("Waiting for heartbeat...")
                self.master.wait_heartbeat()
                print("Heartbeat received from system (system %u component %u)" %
                      (self.master.target_system, self.master.target_component))

                self.connect_status_flag = True
                return True

            except Exception as e:
                print(f"Failed to connect: {e}")
                return False
                
    
    def disconnect_from_uav(self):
        """
        Function for disconnecting UAV from ground station
        :return: bool status that UAV is disconnected from GS
        """
        try:
            if self.master is not None:
                # Close MAVLink connection if open
                self.master.close()
                print("MAVLink connection closed.")

            # Reset internal state
            self.master = None
            self.connect_status_flag = False
            return True

        except Exception as e:
            print(f"Failed to disconnect: {e}")
            return False
        

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
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
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
        while not self.check_if_armed() and time.time() - start_time < self.TIMEOUT:
            time.sleep(0.5)

        # Setting is_armed field
        if self.check_if_armed():
            self.is_armed = True
            print("Armed")
            return True
        else:
            self.is_armed = False
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
        if self.check_if_armed():
            self.is_armed = False
            print("Disarmed")
            return True
        else:
            self.is_armed = True
            print("Cannot disarm")
            return False
        

    @requires_connection
    def get_flight_mode(self):
        # Try to get a fresh HEARTBEAT within timeout
        hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=self.TIMEOUT)
        if hb:
            return mavutil.mode_string_v10(hb)
        # Fallback to last known mode tracked by mavutil (may be stale)
        return getattr(self.master, 'flightmode', None)

    @requires_connection
    def change_flight_mode(self, flight_mode):
        # Validate flight mode
        if flight_mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(flight_mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit()  # TODO replace by exception
        # Get flight mode id
        mode_id = self.master.mode_mapping()[flight_mode]

        # Change flight mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        
        start_time = time.time()
        while self.get_flight_mode() != flight_mode and time.time() - start_time < self.TIMEOUT:
            time.sleep(0.5)
        
        if self.get_flight_mode() == flight_mode:
            print(f"Set flight mode to {flight_mode}")
        else:
            print(f"Can't change flight mode to {flight_mode}")


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
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
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
        # Wait for COMMAND_ACK
        start_time = time.time()
        while time.time() - start_time < self.TIMEOUT:
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
            if not ack:
                continue
            if getattr(ack, "command", None) == mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE:
                ok = ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                print(f"Change altitude ACK: {ack.result} ({'OK' if ok else 'FAIL'})")
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
    def read_data_from_uav(self, poll_window: float = 0.01) -> dict[str, tuple[str, object]]:
        # print(threading.current_thread())
        # self._lock.acquire()
        end = time.time() + poll_window
        # read for 0.05s or until get data
        while time.time() < end:
            msg = self.master.recv_match(blocking=False)
            if not msg:
                # tiny sleep to avoid busy loop
                time.sleep(0.005)
                continue
            # if msg.get_type() == "ATTITUDE":
            #     if self._att_ema_hz < 18:
            #         print("send")
            #         self.set_attitude_rate(self._attitude_rate)

            if msg.get_type() == "BAD_DATA":
                print("Mavlink BAD DATA")
                continue
            self._telemetry_cache.update_from_msg(msg)
        # self._lock.release()
        return self._telemetry_cache.snapshot()


    def do_mavlink_cmd(self):
        pass


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