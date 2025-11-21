import serial
import time
import threading
import keyboard

class RobotController:
    def __init__(self, port: str = "COM5", baudrate: int = 115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            print(f"Connected to robot on {port}")
        except Exception as e:
            raise RuntimeError(f"Could not open serial port {port}: {e}")

        self._shoulder_gain = 0
        self._wrist_gain = 0
        self._shoulder_angle = 0.0
        self._wrist_angle = 0.0
        self._shoulder_offset = 0.0
        self._wrist_offset = 0.0

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
            return round(self._shoulder_angle - self._shoulder_offset, 2)

    def getWristPosition(self) -> float:
        #(0 to 360) anticlockwise is positive direction
        with self._lock:
            return round(self._wrist_angle - self._wrist_offset, 2)
        
    def getShoulderGain(self) -> int:
        return self._shoulder_gain     
    
    def getWristGain(self) -> int:
        return self._wrist_gain

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

    def print_status(self):
        with self._lock:
            
            shoulder = self._shoulder_angle - self._shoulder_offset
            wrist = self._wrist_angle - self._wrist_offset

            line = (
                f"Shoulder: {shoulder:7.2f}° | "
                f"Wrist: {wrist:7.2f}° | "
                f"Gains -> Shoulder: {self._shoulder_gain:4d}, Wrist: {self._wrist_gain:4d}"
            )

        print(line, end="\r", flush=True)

    def calibrate(self):
        print("Use arrow keys to move the robot manually. Press Enter to set encoder offsets.")
        print("Up/Down = Shoulder, Left/Right = Wrist\n")

        last_print = time.time()

        while True:
            if keyboard.is_pressed("up"):
                self.setShoulderGain(100)
            elif keyboard.is_pressed("down"):
                self.setShoulderGain(-100)
            else:
                self.setShoulderGain(0)

            if keyboard.is_pressed("right"):
                self.setWristGain(100)
            elif keyboard.is_pressed("left"):
                self.setWristGain(-100)
            else:
                self.setWristGain(0)

            if time.time() - last_print > 0.1:
                self.print_status()
                last_print = time.time()

            if keyboard.is_pressed("enter"):
                with self._lock:
                    self._shoulder_offset = self._shoulder_angle
                    self._wrist_offset = self._wrist_angle
                self.stop()

                print("\nCalibration complete.")
                print(f"Stored Shoulder offset: {self._shoulder_offset:.2f}")
                print(f"Stored Wrist offset:    {self._wrist_offset:.2f}\n")
                break

            time.sleep(0.01)

    def _pid_control(self, target, current, Kp, Ki, Kd, dt, integral, last_error):
        error = target - current
        integral += error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0

        output = Kp*error + Ki*integral + Kd*derivative
        output = max(min(output, 255), -255)

        return output, integral, error
    
    def moveShoulderPID(self, target_angle, timeout=0.0,
                        Kp=17.5, Ki=0.0, Kd=0.0):

        integral = 0
        last_error = 0
        start = time.time()

        while time.time() - start < timeout:
            current = self.getShoulderPosition()

            dt = 0.01

            output, integral, last_error = self._pid_control(
                target_angle, current, Kp, Ki, Kd, dt, integral, last_error
            )

            self.setShoulderGain(int(output))
            self.print_status()
            time.sleep(dt)

        self.setShoulderGain(0)

    def moveWristPID(self, target_angle, timeout=0.0,
                    Kp=24.0, Ki=0.0, Kd=0.0):

        integral = 0
        last_error = 0
        start = time.time()

        while time.time() - start < timeout:
            current = self.getWristPosition()
            dt = 0.01

            output, integral, last_error = self._pid_control(
                target_angle, current, Kp, Ki, Kd, dt, integral, last_error
            )

            self.setWristGain(int(output))
            self.print_status()
            time.sleep(dt)

        self.setWristGain(0)