
import serial
import numpy as np
import threading
from dataclasses import dataclass, asdict
import json
from time import perf_counter

from linear_mpc import LinearMPC
from PID import PID

def getModelWrist():

    A = np.array([
        [np.float64(1.0), np.float64(0.009956077481910795)],
        [np.float64(-0.0026171816617692836), np.float64(0.12723984685261222)],
    ])

    B = np.array([
        [np.float64(0.0)],
        [np.float64(0.13128606086612418)],
    ])

    C = np.array([
        [np.float64(1.0), np.float64(0.0)],
    ])

    D = np.array([
        [np.float64(0.0)],
    ])

    return A, B, C, D

@dataclass
class Logger:
    angle: list[float]
    vel  : list[float]
    time : list[float]
    power: list[float]

def saveLogg(logg: Logger, title: str):
    # This function is made by ChatGPT

    """Save Logger to JSON file."""
    filename = f"MPC/DATA/{title}.json"
    with open(filename, "w") as f:
        json.dump(asdict(logg), f, indent=4)
    print(f"Saved log to {filename}")

@dataclass
class SerialData:
    position0: float
    position1: float
    power0: int
    power1: int
    positionMode0: bool
    positionMode1: bool
    doResetAngles: bool

class Test:
    def __init__(self):
        self.ser = serial.Serial("COM5", 115200, timeout=0.1)
        self.serialData = SerialData(0.0, 0.0, 0, 0, True, True, False)

        self._prevMotorReading = perf_counter()
        self._currentAngleVel = [0.0, 0.0]
        self._currentAngleValues = [0.0, 0.0]

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.read_motor, daemon=True)
        self._data_lock = threading.Lock()
        self._serial_lock = threading.Lock()

        self._thread.start()

    def get_data(self):
        with self._data_lock:
            return self._currentAngleValues, self._currentAngleVel

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def send_values(self):
        """Send current values to ESP32"""
        msg = (
            f"{self.serialData.position0},{self.serialData.position1},"
            f"{self.serialData.power0},{self.serialData.power1},"
            f"{int(self.serialData.positionMode0)},{int(self.serialData.positionMode1)},{int(self.serialData.doResetAngles)}\n"
        )

        with self._serial_lock:
            if (self.ser is not None):
                self.ser.write(msg.encode("utf-8"))

    def format_line(self, line):
        with self._data_lock:
            prevReading = self._prevMotorReading
            angle = self._currentAngleValues
            vel = self._currentAngleVel

            self._prevMotorReading = perf_counter()

        values = [float(x) for x in line.split(",")][0:4]
        dt = perf_counter() - prevReading

        new_vel0 = (values[0] - angle[0]) / dt
        new_vel1 = (values[1] - angle[1]) / dt


        vel[0] = vel[0] * 0.0 + new_vel0 * 1.0
        vel[1] = vel[1] * 0.0 + new_vel1 * 1.0

        with self._data_lock:
            self._currentAngleValues = values
            self._currentAngleVel = vel

    def read_motor(self):
        while not self._stop_event.is_set():
            try:
                latest = None
                with self._serial_lock:
                    while self.ser.in_waiting:
                        latest = self.ser.readline().decode(errors="ignore").strip()

                if latest is None:
                    return  # nothing to process

                self.format_line(latest)

            except Exception as e:
                print(e)

def printWristStates(t: Test, affix: str = None):
    angle, vel = t.get_data()

    angle = float(angle[0])
    vel = float(vel[0])
    if (affix is not None):
        print(f"Theta: {angle:.2f} | Vel: {vel:.2f} | {affix}")
    else:
        print(f"Theta: {angle:.2f} | Vel: {vel:.2f}")

def setup(t: Test):
    # Shoulder
    t.serialData.position1 =  0.0

    # Wrist
    t.serialData.position0 = 90.0

    t.serialData.power0 = 0.0
    t.serialData.power1 = 0.0
    t.serialData.positionMode0 = True
    t.serialData.positionMode1 = True
    t.send_values()

    angle, vel = t.get_data()
    while (angle < 85.0):
        angle, vel = t.get_data()
        printWristStates(t)

    t0 = perf_counter()
    while (perf_counter() - t0 < 5.0):
        # Wait for 5 seconds.
        printWristStates(t, " Waiting..")

    t.serialData.positionMode0 = False
    t.send_values()

def get_mpc(t: Test):
    A, B, C, D = getModelWrist()
    Q = np.diag([100.0, 1.0])
    R = np.diag([0.1])

    t_step = 0.01

    angle, vel = t.get_data()
    x0 = np.array([angle[0], vel[0]]).T
    print(f"x0: {x0}")
    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=10, t_step=t_step)
    mpc_system.init_controller(x0)
    print("Setup complete.")
    return mpc_system

def main():
    t = Test()

    setup(t)
    mpc_system = get_mpc(t)
    pid = PID(5.0, 0.0, 0.0, 0.0, (-255, 255), 0.01)

    mode = "MPC"
    t0_glob = perf_counter()
    max_run_time = 10.0
    logg = Logger([], [], [], [])
    timeSinceStart = 0.0

    if (mode != "PID" and mode != "MPC"): raise ValueError("Invalid mode.")
    while (timeSinceStart < max_run_time):
        timeSinceStart = perf_counter() - t0_glob
        angle, vel = t.get_data()

        if (mode == "PID"):
            t0 = perf_counter()
            u0 = pid.update(angle[0])
            t1 = perf_counter()
        elif (mode == "MPC"):
            x_current = np.array([angle[0], vel[0]]).T

            t0 = perf_counter()
            u0 = mpc_system.step(x_current)
            t1 = perf_counter()

        t.serialData.power0 = int(u0)
        t.send_values()
        printWristStates(t, f" Power: {int(u0)} | Time: {timeSinceStart:.1f} | Steptime: {t1 - t0}")

        logg.angle.append(angle[0])
        logg.vel.append(vel[0])
        logg.time.append(perf_counter())
        logg.power.append(float(u0))
    print(f"Finished in {perf_counter() - t0_glob:.2f} seconds. Datapoints: {len(logg.angle)}. Datapoints per seconds = {len(logg.angle) / (perf_counter() - t0_glob)}.")
    saveLogg(logg, f"{mode}Logg")


if __name__ == "__main__": main()

