import sys
import pygame
import threading

class Joystick:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.running = False
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

    def get_pwm_joystick(self, j_value):
        return int(((j_value + 1) / 2) * (2000 - 1000) + 1000)

    def check_joystick_connection(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print('No joystick connected')
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print("Find Joystick:", self.joystick.get_name())

    def get_rc_mapping_value(self, rc_values):
        for i, value in enumerate(rc_values, start=1):
            self.rc_mapping_dict["RC"][i]["default_value"] = value
            self.rc_mapping_dict["RC"][i]["pwm_value"] = self.get_pwm_joystick(value)

        values = [f"{v['name']}: {v['default_value']:.2f}, {v['pwm_value']}" for v in self.rc_mapping_dict["RC"].values()]
        print(" | ".join(values), end='\r')

    def run_joystick(self):
        self.running = True
        self.check_joystick_connection()

        if self.joystick is None:
            print("Joystick not found")
            return

        while self.running:
            pygame.event.pump()
            rc_values = [self.joystick.get_axis(i) for i in range(8)]
            self.get_rc_mapping_value(rc_values)
            pygame.time.wait(20)

    def start(self):
        thread = threading.Thread(target=self.run_joystick, daemon=True)
        thread.start()

    def stop(self):
        self.running = False
        print("\n[Joystick] Stopped.")


if __name__ == '__main__':
    from time import sleep
    joystick = Joystick()
    joystick.start()

    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        joystick.stop()
        sys.exit(0)
