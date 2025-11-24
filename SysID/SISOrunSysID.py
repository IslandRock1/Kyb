import time
import numpy as np
import random
from PythonCode.robotController import RobotController
from SysID.SISOdataCollector import SystemIdentificationDataCollector

def apply_gain(robot, joint, value, max_gain=255):
    """Map normalized input [-1,1] to robot gain and apply."""
    gain = int(value * max_gain)
    if joint == "shoulder":
        robot.setShoulderGain(gain)
    elif joint == "wrist":
        robot.setWristGain(gain)
    else:
        raise ValueError("Joint must be 'shoulder' or 'wrist'")

def ramp_positive_start(robot, joint, duration, amplitude, sampling_time):
    steps = int(duration / sampling_time)
    for i in range(steps):
        t = i * sampling_time
        if t < duration / 3:
            u = (amplitude * 3 * t) / duration
        elif t < 2 * duration / 3:
            u = amplitude - (amplitude * 6 * (t - duration / 3)) / duration
        else:
            u = -amplitude + (amplitude * 3 * (t - 2 * duration / 3)) / duration
        apply_gain(robot, joint, u)
        time.sleep(sampling_time)

def ramp_negative_start(robot, joint, duration, amplitude, sampling_time):
    steps = int(duration / sampling_time)
    for i in range(steps):
        t = i * sampling_time
        if t < duration / 3:
            u = -(amplitude * 3 * t) / duration
        elif t < 2 * duration / 3:
            u = -amplitude + (amplitude * 6 * (t - duration / 3)) / duration
        else:
            u = amplitude - (amplitude * 3 * (t - 2 * duration / 3)) / duration
        apply_gain(robot, joint, u)
        time.sleep(sampling_time)

def sine_motion(robot, joint, duration, amplitude, freq, sampling_time):
    steps = int(duration / sampling_time)
    for i in range(steps):
        t = i * sampling_time
        u = amplitude * np.sin(2 * np.pi * freq * t)
        apply_gain(robot, joint, u)
        time.sleep(sampling_time)

def sine_chirp(robot, joint, duration, amplitude, f_start, f_end, sampling_time):
    steps = int(duration / sampling_time)
    for i in range(steps):
        t = i * sampling_time
        #linear chirp frequency law
        freq = f_start + (f_end - f_start) * (t / duration)
        u = amplitude * np.sin(2 * np.pi * freq * t)
        apply_gain(robot, joint, u)
        time.sleep(sampling_time)

def prbs_motion(robot, joint, duration, amplitude, step_duration, sampling_time):
    n_steps = int(duration / step_duration)
    samples_per_step = int(step_duration / sampling_time)

    for step in range(n_steps):
        u = random.choice([-amplitude, amplitude])
        for _ in range(samples_per_step):
            apply_gain(robot, joint, u)
            time.sleep(sampling_time)

def small_oscillation(robot, joint, duration, amplitude, freq, sampling_time):
    steps = int(duration / sampling_time)
    for i in range(steps):
        t = i * sampling_time
        u = amplitude * np.sin(2 * np.pi * freq * t)
        apply_gain(robot, joint, u)
        time.sleep(sampling_time)

if __name__ == "__main__":
    robot = RobotController()
    joint = "wrist"
    data_file = f"{joint}_sysid_rampfocus_1.csv"
    collector = SystemIdentificationDataCollector(robot, joint, data_file)

    RANDOM_SEED = 111

    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)

    robot.calibrate()
    collector.startDataCollection()

    collector.setMovementDescription("Ramp positive start")
    ramp_positive_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp negative start")
    ramp_negative_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp positive start")
    ramp_positive_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp negative start")
    ramp_negative_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp positive start")
    ramp_positive_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp negative start")
    ramp_negative_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp positive start")
    ramp_positive_start(robot, joint, 24, 1.0, 0.1)

    collector.setMovementDescription("Ramp negative start")
    ramp_negative_start(robot, joint, 24, 1.0, 0.1)


    """
    collector.setMovementDescription("Small oscillation")
    small_oscillation(robot, joint, 10, 0.10, 1.0, 0.01)


    sine_frequencies = [0.2, 0.6, 1.0, 2.0]
    for f in sine_frequencies:
        collector.setMovementDescription(f"Sine {f} Hz")
        sine_motion(robot, joint, 3, 0.8, f, 0.01)
        apply_gain(robot, joint, 0)

    collector.setMovementDescription("Chirp sweep")
    sine_chirp(robot, joint, 20, 0.7, 0.05, 2.0, 0.01)

    collector.setMovementDescription("PRBS motion")
    prbs_motion(robot, joint, 30, 0.75, 0.7, 0.01)
    """
    collector.setMovementDescription("No Movement")
    apply_gain(robot, joint, 0)
    collector.stopDataCollection()
    robot.moveShoulderPID(0, 10.0)
    robot.moveWristPID(0, 10.0)
    robot.stop()
    robot.close()

    print("System identification data collection complete.")