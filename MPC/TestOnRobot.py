import time

import serial
import numpy as np
import threading
from dataclasses import dataclass, asdict
import json
from time import perf_counter, sleep

from utils import getCompleteModel, SimpleFilter
from linear_mpc import LinearMPC
from PID import PID

@dataclass
class Logger:
    angle0: list[float]
    angle1: list[float]
    vel0  : list[float]
    vel1  : list[float]
    time  : list[float]
    power0: list[float]
    power1: list[float]

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
        self._currentPower = [0.0, 0.0]

        self.filter0 = SimpleFilter(method="lowpass_angle", alpha=0.05)
        self.filter1 = SimpleFilter(method="lowpass_angle", alpha=0.05)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.read_motor, daemon=True)
        self._data_lock = threading.Lock()
        self._serial_lock = threading.Lock()

        self.espMsg = ""
        self.espMsgLock = threading.Lock()

        self.numDataReadings = 0

        self._thread.start()

    def get_data(self):
        with self._data_lock:
            return self._currentAngleValues, self._currentAngleVel, self._currentPower

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def send_values(self):
        """Send current values to ESP32"""
        with self.espMsgLock:
            self.espMsg = (
                f"{self.serialData.position0},{self.serialData.position1},"
                f"{self.serialData.power0},{self.serialData.power1},"
                f"{int(self.serialData.positionMode0)},{int(self.serialData.positionMode1)},{int(self.serialData.doResetAngles)}\n"
            )

    def format_line(self, line):
        with self._data_lock:
            prevReading = self._prevMotorReading
            angle = self._currentAngleValues
            vel = self._currentAngleVel

            self._prevMotorReading = perf_counter()

        try:
            values = [float(x) for x in line.split(",")][0:4]
            dt = perf_counter() - prevReading

            vel[0] = self.filter0.update(values[0], dt)
            vel[1] = self.filter1.update(values[1], dt)

            angles = [values[0], values[1]]
            power = [values[2], values[3]]

            with self._data_lock:
                self._currentAngleValues = angles
                self._currentAngleVel = vel
                self._currentPower = power
                self.numDataReadings += 1
        except Exception as e:
            print(e)


    def read_motor(self):
        while not self._stop_event.is_set():
            try:
                with self.espMsgLock:
                    msg = self.espMsg

                latest = None
                with self._serial_lock:
                    self.ser.write(msg.encode("utf-8"))

                    while self.ser.in_waiting:
                        latest = self.ser.readline().decode(errors="ignore").strip()

                if latest is None:
                    continue  # nothing to process

                self.format_line(latest)

            except Exception as e:
                print(e)

def printStates(t: Test, affix: str = None):
    angle, vel, power = t.get_data()

    if (affix is not None):
        print(f"Theta: {angle} | Vel: {vel} | {affix}")
    else:
        print(f"Theta: {angle} | Vel: {vel} | {power}")

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

    time.sleep(1.0)
    t.serialData.position1 = 90.0
    t.send_values()

    angle, vel, _ = t.get_data()
    while (abs(angle[0]) < 80.0) or (abs(angle[1]) < 80.0):
        angle, vel, _ = t.get_data()
        printStates(t)

    t0 = perf_counter()
    while (perf_counter() - t0 < 5.0):
        # Wait for 5 seconds.
        printStates(t, " Waiting..")

    t.serialData.positionMode0 = False
    t.serialData.positionMode1 = False
    t.send_values()

def get_mpc(t: Test):
    A, B, C, D = getCompleteModel()
    Q = np.diag([1000000.0, 1000000.0, 10000.0, 10000.0])
    R = np.diag([0.000001, 0.000001])

    t_step = 0.01

    angle, vel, _ = t.get_data()

    physicalStates = np.array([angle[0], vel[0], angle[1], vel[1]]).T
    x0 = (np.linalg.inv(C) @ physicalStates)
    print(f"x0: {x0}")
    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=10, t_step=t_step)
    mpc_system.init_controller(x0)
    print("Setup complete.")
    return mpc_system

def main():
    t = Test()
    _, _, C, _ = getCompleteModel()

    setup(t)
    mpc_system = get_mpc(t)
    pid0 = PID(40.0, 0.0, 0.0, 0.0, (-255, 255), 0.01)
    pid1 = PID(40.0, 0.0, 0.0, 0.0, (-255, 255), 0.01)

    mode = "MPC"
    t0_glob = perf_counter()
    max_run_time = 10.0
    logg = Logger([], [], [], [], [], [], [])
    timeSinceStart = 0.0

    if (mode != "PID" and mode != "MPC"): raise ValueError("Invalid mode.")
    while (timeSinceStart < max_run_time):
        t0_iter = perf_counter()

        timeSinceStart = perf_counter() - t0_glob
        angle, vel, _ = t.get_data()

        if (mode == "PID"):
            t0 = perf_counter()
            u0 = pid0.update(angle[0])
            u1 = pid1.update(angle[1])
            u0 = np.array([u0, u1])
            t1 = perf_counter()
        elif (mode == "MPC"):
            physicalStates = np.array([angle[0], vel[0], angle[1], vel[1]]).T
            x_current = (np.linalg.inv(C) @ physicalStates)

            t0 = perf_counter()
            u0 = mpc_system.step(x_current)
            t1 = perf_counter()

        t.serialData.power0 = int( u0[0] * 0.5)
        t.serialData.power1 = int(-u0[1] * 0.5)
        t.send_values()
        printStates(t, f" Power: {u0} | Time: {timeSinceStart:.1f} | Steptime: {t1 - t0}")

        logg.angle0.append(angle[0])
        logg.angle1.append(angle[1])
        logg.vel0.append(vel[0])
        logg.vel1.append(vel[1])
        logg.time.append(perf_counter())
        logg.power0.append(float(u0[0]))
        logg.power1.append(float(u0[1]))

        sleep_time = 0.01 - (perf_counter() - t0_iter)
        if (sleep_time < 0.0):
            print(f"Sleeptime: {sleep_time}")
            continue
        if (sleep_time > 0.01):
            print(f"Sleeptime: {sleep_time}")
            continue
        time.sleep(sleep_time)
    print(f"Finished in {perf_counter() - t0_glob:.2f} seconds. Datapoints: {len(logg.angle0)}. Datapoints per seconds = {len(logg.angle0) / (perf_counter() - t0_glob)}.")
    saveLogg(logg, f"{mode}Logg")

    print(f"Num readings from esp: {t.numDataReadings}.")


if __name__ == "__main__": main()

