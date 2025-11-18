import time
import random
from robotController import RobotController
from systemIdentificationDataCollector import SystemIdentificationDataCollector

class BlockSystemIdentification:
    def __init__(self, robot: RobotController, joint: str, filename: str, blocks: int):
        if blocks % 2 != 0:
            raise ValueError("Number of blocks must be even to balance methods")
            
        self.robot = robot
        self.joint = joint.lower()
        self.blocks = blocks
        self.filename = filename
        self.data_collector = SystemIdentificationDataCollector(robot, joint, filename)
        self.max_angle = 270.0  #maximum deviation from zero
        self.min_angle = -270.0

    def _random_gain_and_duration(self):
        gain = random.randint(30, 255)
        duration = random.uniform(0.5, 3.0)
        direction = random.choice([-1, 1])
        return direction * gain, duration

    def _get_joint_position(self):
        if self.joint == "shoulder":
            return self.robot.getShoulderPosition()
        else:
            return self.robot.getWristPosition()

    def _apply_gain(self, gain):
        if self.joint == "shoulder":
            self.robot.setShoulderGain(gain)
        else:
            self.robot.setWristGain(gain)

    def _stop_joint(self):
        self._apply_gain(0)

    def _run_block(self, method: str, duration: float = 60.0):
        start_time = time.time()
        print(f"Starting {method} block. Moving {self.joint} randomly for {duration:.1f}s.")
        self._stop_joint()
        self.data_collector.setMovementDescription(f"{method} block start: zeroing")
        self.data_collector.startDataCollection()

        elapsed = 0.0

        while elapsed < duration:
            gain, move_duration = self._random_gain_and_duration()
            
            #check safety
            current_pos = self._get_joint_position()
            if (current_pos >= self.max_angle and gain > 0) or (current_pos <= self.min_angle and gain < 0):
                print(f"safety stop: joint at {current_pos:.1f}°, reversing gain {gain} -> {-gain}")
                gain = -gain

            #adjusts move_duration if it would exceed block duration
            stop_time = 1.0 if method == "StopStart" else 0.0
            if elapsed + move_duration + stop_time > duration:
                move_duration = duration - elapsed - stop_time
                if move_duration <= 0:
                    break

            self.data_collector.setMovementDescription(f"{method} moving at gain {gain}")
            print(f"[{method}] Applying gain {gain} for {move_duration:.2f}s (current pos {current_pos:.1f}°)")

            self._apply_gain(gain)
            time.sleep(move_duration)

            if method == "StopStart":
                self._stop_joint()
                time.sleep(1.0)

            elapsed = time.time() - start_time

        print(f"{method} block finished. Stopping {self.joint} and returning to zero.")
        self._stop_joint()
        self.data_collector.stopDataCollection()

    def run(self):
        for i in range(self.blocks):
            method = "QuickChange" if i % 2 == 0 else "StopStart"
            print(f"\n=== Starting Block {i+1}/{self.blocks} ({method}) ===")
            self._run_block(method)
        print("\nSystem identification run complete!")
