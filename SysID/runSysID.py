import time
import numpy as np
import random
from robotController import RobotController
from SysID.dataCollector import DataCollector

maxGain = int(0.75 * 255)
dt = 0.01
shoulderGainRecovery = 191
recoveryTime = 7.5

def rampPosNeg(t, duration, amplitude=0.75):
    q = duration / 4
    if t < q: return amplitude * (t/q)
    if t < 2*q: return amplitude * (1 - (t-q)/q)
    if t < 3*q: return -amplitude * ((t-2*q)/q)
    return -amplitude * (1 - (t-3*q)/q)

class Prbs:
    def __init__(self, interval=0.3, amplitude=0.75):
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

def sineWave(t, freq, amplitude=0.75):
    return amplitude * np.sin(2*np.pi*freq*t)

def chirpLinear(t, duration, f0, f1, amplitude=0.75):
    f = f0 + (f1 - f0)*(t/duration)
    return amplitude * np.sin(2*np.pi*f*t)

def runPattern(robot, duration, shFun, wrFun, desc, collector):
    collector.setDescription(desc)
    steps = int(duration / dt)
    recovering = False
    recoverUntil = 0
    recoverDir = 0

    for i in range(steps):
        t = i*dt
        shPos = robot.getShoulderPosition()

        if not recovering:
            if shPos > 270:
                recovering = True
                recoverDir = -1
                recoverUntil = time.time() + recoveryTime
            elif shPos < -270:
                recovering = True
                recoverDir = +1
                recoverUntil = time.time() + recoveryTime

        if recovering:
            if time.time() < recoverUntil:
                robot.setShoulderGain(recoverDir * shoulderGainRecovery)
                robot.setWristGain(0)
                time.sleep(dt)
                continue
            recovering = False

        sh = shFun(t)
        wr = wrFun(t)
        shGain = int(np.clip(sh * maxGain, -maxGain, maxGain))
        wrGain = int(np.clip(wr * maxGain, -maxGain, maxGain))
        robot.setShoulderGain(shGain)
        robot.setWristGain(wrGain)
        time.sleep(dt)

if __name__ == "__main__":
    random.seed(69)
    np.random.seed(69)

    robot = RobotController("COM5")
    robot.calibrate()

    collector = DataCollector(robot, "MIMO_full.csv")
    collector.startDataCollection()

    rampVariants = [
        ("RampA05", 10, lambda t, d=10: rampPosNeg(t, d, 0.50)),
        ("RampA075", 10, lambda t, d=10: rampPosNeg(t, d, 0.75)),
    ]

    sineVariants = [
        ("Sine04", 10, lambda t: sineWave(t, 0.4)),
        ("Sine08", 10, lambda t: sineWave(t, 0.8)),
        ("Sine12", 10, lambda t: sineWave(t, 1.2)),
    ]

    prbsVariants = [
        ("Prbs02", 10, Prbs(0.2)),
        ("Prbs03", 10, Prbs(0.3)),
        ("Prbs05", 10, Prbs(0.5)),
    ]

    chirpVariants = [
        ("ChirpSlow",10, lambda t, d=10: chirpLinear(t, d, 0.05, 1.0)),
        ("ChirpMid", 10, lambda t, d=10: chirpLinear(t, d, 0.1, 2.0)),
        ("ChirpFast",10, lambda t, d=10: chirpLinear(t, d, 0.3, 4.0)),
    ]

    movementSets = [
        ("Ramp", rampVariants),
        ("Sine", sineVariants),
        ("PRBS", prbsVariants),
        ("Chirp", chirpVariants),
    ]

    allVariants = []
    for _, lst in movementSets:
        for v in lst:
            allVariants.append(v)

    total = len(allVariants)*2 + len(allVariants)*len(allVariants)
    idx = 1

    for name, dur, fun in allVariants:
        print(f"({idx}/{total}) ShoulderOnly_{name}")
        runPattern(robot, dur, fun, lambda t: 0, f"SOnly_{name}", collector)
        idx += 1

    for name, dur, fun in allVariants:
        print(f"({idx}/{total}) WristOnly_{name}")
        runPattern(robot, dur, lambda t: 0, fun, f"WOnly_{name}", collector)
        idx += 1

    for shName, shDur, shFun in allVariants:
        for wrName, wrDur, wrFun in allVariants:
            d = max(shDur, wrDur)
            label = f"{shName}/{wrName}"
            print(f"({idx}/{total}) {label}")
            runPattern(robot, d, shFun, wrFun, label, collector)
            idx += 1

    collector.setDescription("Idle")
    robot.stop()
    collector.stopDataCollection()

    robot.moveShoulderPID(0, 5)
    robot.moveWristPID(0, 5)
    robot.stop()
    robot.close()