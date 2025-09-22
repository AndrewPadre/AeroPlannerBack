import threading
import time
import socket
from app.settings import settings


class GimbalViewPro:
    def __init__(self):
        self.gimbal_url = settings.GIMBAL_VIEW_PRO_URL
        self.gimbal_port = settings.GIMBAL_VIEW_PRO_PORT
        self.heartbeat_bytes = bytes.fromhex(settings.VIEW_PRO_HEARTBEAT)
        self.timeout_sec = 0.001  # 1 ms
        self.interval_sec = 0.08  # 80 ms
        self.sock = None
        self.connected = False

        self.azimuth = 0.0
        self.pitch = 0.0

        self.gimbal_data_dict = {
            "connected": self.connected,
            "azimuth_deg": self.azimuth,
            "latitude_deg": None,
            "longitude_deg": None,
            "roll_deg": None,
            "pitch_deg": self.pitch,
        }

    def start(self):
        thread = threading.Thread(target=self.run, daemon=True)
        thread.start()

    def run(self):
        self.connect()
        while True:
            if not self.connected or not self.check_connection():
                print("Connection lost. Attempting to reconnect...")
                self.cleanup()
                time.sleep(1)
                self.connect()

            self.send_heartbeat()
            self.receive_response()
            time.sleep(self.interval_sec)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout_sec)
            self.sock.connect((self.gimbal_url, self.gimbal_port))
            self.connected = True
            print("TCP connection established.")
        except Exception as e:
            print(f"Failed to connect: {e}")
            self.connected = False

    def send_heartbeat(self):
        if self.connected:
            try:
                self.sock.sendall(self.heartbeat_bytes)
            except Exception as e:
                print(f"Error sending heartbeat: {e}")
                self.connected = False

    def receive_response(self):
        if not self.connected:
            self.output_orientation(0.0, 0.0)
            return

        try:
            response = self.sock.recv(1024)
            if response:
                hex_response = response.hex().upper()
                print(f"Received response: {hex_response}")
                az, pitch = self.extract_azimuth_and_pitch(hex_response)

                if az is not None and pitch is not None:
                    self.output_orientation(az, pitch)
                else:
                    print("⚠️ Response does not contain azimuth/pitch.")
            else:
                print("⚠️ Empty response.")
        except socket.timeout:
            pass
        except Exception as e:
            print(f"⚠️ Error receiving data: {e}")
            self.connected = False

    def extract_azimuth_and_pitch(self, message):
        if message.startswith("55AADC") and len(message) > 70:
            try:
                hex_az = message[60:64]
                dec_az = int(hex_az, 16)
                az = dec_az * 360.0 / 65536.0

                hex_pitch = message[64:68]
                dec_pitch = int(hex_pitch, 16)
                pitch = dec_pitch * 360.0 / 65536.0

                return az, pitch
            except Exception as e:
                print(f"Error processing azimuth/pitch: {e}")
        return None, None

    def output_orientation(self, az, pitch):
        self.azimuth = az
        self.pitch = pitch
        print(f"Azimuth: {az:.2f}° | Pitch: {pitch:.2f}°")

    def check_connection(self):
        try:
            self.sock.send(b'')  # harmless ping
            return True
        except:
            return False

    def cleanup(self):
        try:
            if self.sock:
                self.sock.close()
        except:
            pass
        print("Connection closed.")
        self.sock = None
        self.connected = False

if __name__ == "__main__":
    g = GimbalViewPro()
    g.run()
