import sys
import time
import pygame
import threading


class Joystick:
    def __init__(self):
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

    def do_default_rc_mapping_dict(self):
        for key in self.rc_mapping_dict:
            for channel in self.rc_mapping_dict.get("RC", {}):
                channel_data = self.rc_mapping_dict["RC"][channel]
                channel_data["default_value"] = None
                channel_data["pwm_value"] = None

    def joystick_connection_status(self) -> bool:
        pygame.joystick.quit()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        #print(f"[DEBUG] Joystick count: {joystick_count}")
        if joystick_count == 0:
            #print('No joystick connected')
            self.joystick_name = None
            return False
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_name = self.joystick.get_name()
            #print("Joystick connected:", self.joystick_name)
            return True

    def is_joystick_connected(self) -> bool:
        return pygame.joystick.get_count() > 0


    def get_pwm_joystick(self, j_value):
        return int(((j_value + 1) / 2) * (2000 - 1000) + 1000)


    def get_rc_mapping_value(self, rc_values):
        for i, value in enumerate(rc_values, start=1):
            self.rc_mapping_dict["RC"][i]["default_value"] = value
            self.rc_mapping_dict["RC"][i]["pwm_value"] = self.get_pwm_joystick(value)

    def start_joystick_thread(self):
        thread  = threading.Thread(target=self.run)
        thread.start()

    def start_joystick(self):
        self.running = True

    def stop_joystick(self):
        self.running = False
        self.do_default_rc_mapping_dict()

    def run(self):
        while True:
            if not self.joystick_connection_status():
                time.sleep(1)
                continue

            try:
                while self.is_joystick_connected():
                    pygame.event.pump()

                    if self.running:
                        try:
                            rc_values = [self.joystick.get_axis(i) for i in range(8)]
                            self.get_rc_mapping_value(rc_values)
                        except IndexError:
                            print("Joystick does not have 8 axes", end='\r')

                        pygame.time.wait(20)
                    else:
                        time.sleep(0.02)

                print("Joystick disconnected")
                self.do_default_rc_mapping_dict() # reset dict to default
                self.joystick = None

            except pygame.error as e:
                print(f"Joystick error: {e}")
                self.joystick = None
                print("Rechecking for joystick...")

if __name__ == "__main__":
    joystick = Joystick()
    joystick.run()

