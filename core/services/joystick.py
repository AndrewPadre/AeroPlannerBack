
import time
import pygame
import threading


class RemoteController:
    def __init__(self):
        self.start_rc_flag = False
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.rc_pwm_default_dict = {"min": 1000, "trim": 1500, "max": 2000}
        self.rc_mapping_dict = {
            "RC": {
                1: {"name": "roll", "default_value": None, "pwm_value": None},
                2: {"name": "pitch", "default_value": None, "pwm_value": None},
                3: {"name": "throttle", "default_value": None, "pwm_value": None},
                4: {"name": "yaw", "default_value": None, "pwm_value": None},
                5: {"name": "empty", "default_value": None, "pwm_value": None},
                6: {"name": "empty", "default_value": None, "pwm_value": None},
                7: {"name": "modes", "default_value": None, "pwm_value": None},
                8: {"name": "paraschute", "default_value": None, "pwm_value": None},
            }
        }
        self.running = False
        self.joystick_name = None
        self._stop = threading.Event()

    def do_default_rc_mapping_dict(self) -> dict:
        for _ in self.rc_mapping_dict:
            for channel in self.rc_mapping_dict.get("RC", {}):
                channel_data = self.rc_mapping_dict["RC"][channel]
                channel_data["default_value"] = None
                channel_data["pwm_value"] = None
                
        return self.rc_mapping_dict

    def connect(self) -> bool:
        pygame.joystick.quit()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        print(f"[DEBUG] Joystick count: {joystick_count}")
        if joystick_count == 0:
            print('No joystick connected')
            self.joystick_name = None
            return False
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_name = self.joystick.get_name()
            print("Joystick connected:", self.joystick_name)
            return True
        
    def disconnect(self):
        pygame.joystick.quit()

    def is_joystick_connected(self) -> bool:
        return pygame.joystick.get_count() > 0

    def float_to_pwm(self, j_value) -> int:
        return int(((j_value + 1) / 2) * (2000 - 1000) + 1000)

    def get_rc_mapping_value(self, rc_values):
        for i, value in enumerate(rc_values, start=1):
            self.rc_mapping_dict["RC"][i]["default_value"] = value
            self.rc_mapping_dict["RC"][i]["pwm_value"] = self.float_to_pwm(value)
        return self.rc_mapping_dict

    def get_snapshot(self) -> dict:
        if not self.is_joystick_connected():
            return self.do_default_rc_mapping_dict()
        pygame.event.pump()
        rc_values = [self.joystick.get_axis(i) for i in range(8)]
        mapped_rc_values = self.get_rc_mapping_value(rc_values)
        return mapped_rc_values



if __name__ == "__main__":
    joystick = RemoteController()
    joystick.connect()
    while True:
        print(joystick.get_snapshot())
        time.sleep(0.1)

