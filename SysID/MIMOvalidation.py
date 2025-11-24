import time
import numpy as np
import random
from robotController import RobotController
from MIMOdataCollector import MIMOSystemIdentificationDataCollector

DT = 0.01
MAX_SAFE_GAIN = int(0.75 * 255)

def sine(t, freq=0.5, amp=0.5):
    return amp * np.sin(2*np.pi*freq*t)

class Prbs:
    def __init__(self, interval=0.3, amplitude=0.5):
        self.interval = interval
        self.amplitude = amplitude
        self.nextSwitch = 0
        self.val = 0
        self.lastT = -1
    def __call__(self, t):
        if t < self.lastT:
            self.nextSwitch = 0
            self.val = random.choice([-self.amplitude, self.amplitude])
        self.lastT = t
        if t >= self.nextSwitch:
            self.val = random.choice([-self.amplitude, self.amplitude])
            self.nextSwitch = t + self.interval
        return self.val

def step_full(t):
    return 1.0

def ramp_full(t, duration=10):
    return np.clip(t / duration, 0, 1)

def runPattern(robot, duration, shFun, wrFun, label, collector):
    collector.setDescription(label)
    steps = int(duration / DT)
    for i in range(steps):
        t = i * DT
        sh = int(np.clip(shFun(t) * MAX_SAFE_GAIN, -MAX_SAFE_GAIN, MAX_SAFE_GAIN))
        wr = int(np.clip(wrFun(t) * MAX_SAFE_GAIN, -MAX_SAFE_GAIN, MAX_SAFE_GAIN))
        robot.setShoulderGain(sh)
        robot.setWristGain(wr)
        time.sleep(DT)

if __name__ == "__main__":
    random.seed(42)
    np.random.seed(42)

    robot = RobotController("COM5")
    robot.calibrate()
    collector = MIMOSystemIdentificationDataCollector(robot, "validation_2.csv")
    collector.startDataCollection()

    tests = [
        ("Shoulder_Step", 5, lambda t: step_full(t), lambda t: 0),
        ("Wrist_Step",    5, lambda t: 0,       lambda t: step_full(t)),
        ("Shoulder_Ramp", 5, lambda t: ramp_full(t, 10), lambda t: 0),
        ("Wrist_Ramp",    5, lambda t: 0,       lambda t: ramp_full(t, 10)),
    ]

    for name, dur, shF, wrF in tests:
        print("Running:", name)
        runPattern(robot, dur, shF, wrF, name, collector)

    collector.setDescription("Idle")
    collector.stopDataCollection()
    robot.setShoulderGain(0)
    robot.setWristGain(0)
    robot.moveShoulderPID(0, 5)
    robot.moveWristPID(0, 5)
    robot.close()
