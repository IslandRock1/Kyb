import serial
import time
import threading
import keyboard

class RobotController:
    def __init__(self, port: str = "COM8", baudrate: int = 115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            print(f"Connected to robot on {port}")
        except Exception as e:
            raise RuntimeError(f"Could not open serial port {port}: {e}")

        self._shoulder_gain = 0
        self._wrist_gain = 0
        self._shoulder_angle = 0.0
        self._wrist_angle = 0.0

        self._lock = threading.Lock()
        self.running = True

        threading.Thread(target=self._read_serial_loop, daemon=True).start()

    def _send(self):
        msg = f"0,0,{self._wrist_gain},{self._shoulder_gain},0,0,0\n"
        self.ser.write(msg.encode("utf-8"))

    def _read_serial_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    parts = line.split(",")
                    if len(parts) >= 2:
                        with self._lock:
                            self._wrist_angle = float(parts[0])
                            self._shoulder_angle = float(parts[1])
            except Exception:
                pass

    def setShoulderGain(self, gain: int):
        #(-255 to 255) zero position is at 3 oclock
        self._shoulder_gain = -int(gain)
        self._send()

    def setWristGain(self, gain: int):
        #(-255 to 255) zero position is X+ towards table, and Y+ is upwards
        self._wrist_gain = int(gain)
        self._send()

    def getShoulderPosition(self) -> float:
        #(0 to 360) anticlockwise is positive direction, zero position is at 3 oclock
        with self._lock:
            return self._shoulder_angle

    def getWristPosition(self) -> float:
        #(0 to 360) anticlockwise is positive direction
        with self._lock:
            return self._wrist_angle

    def stop(self):
        self._wrist_gain = 0
        self._shoulder_gain = 0
        self._send()

    def close(self):
        self.running = False
        self.stop()
        time.sleep(0.1)
        if self.ser.is_open:
            self.ser.close()
        print("Connection closed.")

    def calibrate(self):
        print("Use arrow keys to move the robot manually. Press Enter to set encoder offsets.")
        print("Up/Down = Shoulder, Left/Right = Wrist")

        while True:
            if keyboard.is_pressed("up"):
                self.setShoulderGain(50)
            elif keyboard.is_pressed("down"):
                self.setShoulderGain(-50)
            else:
                self.setShoulderGain(0)

            if keyboard.is_pressed("right"):
                self.setWristGain(50)
            elif keyboard.is_pressed("left"):
                self.setWristGain(-50)
            else:
                self.setWristGain(0)

            if keyboard.is_pressed("enter"):
                with self._lock:
                    self._shoulder_offset = self._shoulder_angle
                    self._wrist_offset = self._wrist_angle
                self.stop()
                print(f"Calibration done. Shoulder offset: {self._shoulder_offset:.2f}, Wrist offset: {self._wrist_offset:.2f}")
                break

            time.sleep(0.05)

if __name__ == "__main__":
    robot = RobotController("COM8", 115200)
    robot.calibrate()

    time.delay(10)

    robot.close()
    print("Test finished.")