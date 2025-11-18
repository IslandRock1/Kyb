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
        self.max_angle = 270.0
        self.min_angle = -270.0
        self._next_direction = 1

    def _random_gain_and_duration(self):
        gain = random.randint(30, 255) * self._next_direction
        self._next_direction *= -1
        duration = random.uniform(0.5, 3.0)
        return gain, duration

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

    def _print_with_timestamp(self, message: str, start_time: float):
        elapsed = time.time() - start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        print(f"[{minutes:02d}:{seconds:02d}] {message}")

    def _run_block(self, method: str, duration: float, total_start_time: float):
        block_start_time = time.time()
        self._stop_joint()
        self.data_collector.setMovementDescription(f"{method} block start: zeroing")
        self._print_with_timestamp(f"{method} block start: zeroing", total_start_time)

        elapsed = 0.0

        while elapsed < duration:
            gain, move_duration = self._random_gain_and_duration()
            remaining = move_duration
            current_pos = self._get_joint_position()

            elapsed = time.time() - block_start_time
            self.data_collector.setMovementDescription(
                f"{method} gain {gain} for {move_duration:.2f}s"
            )
            self._print_with_timestamp(
                f"{method} gain {gain} applied at pos {current_pos:.1f}° for {move_duration:.2f}s",
                total_start_time
            )

            step = 0.05
            while remaining > 0 and elapsed < duration:
                current_pos = self._get_joint_position()

                if (current_pos >= self.max_angle and gain > 0) or (current_pos <= self.min_angle and gain < 0):
                    gain = -gain
                    self._print_with_timestamp(
                        f"safety stop: joint at {current_pos:.1f}°, reversing gain -> {gain}",
                        total_start_time
                    )
                    self._apply_gain(gain)
                else:
                    self._apply_gain(gain)

                sleep_time = min(step, remaining, duration - elapsed)
                time.sleep(sleep_time)
                remaining -= sleep_time
                elapsed = time.time() - block_start_time

            if method == "StopStart" and elapsed < duration:
                self._stop_joint()
                time.sleep(1.0)
                elapsed = time.time() - block_start_time

        self._stop_joint()
        self._print_with_timestamp(
            f"{method} block finished. Stopping.", total_start_time
        )

    def run(self):
        total_start_time = time.time()
        self.data_collector.startDataCollection()

        for i in range(self.blocks):
            method = "QuickChange" if i % 2 == 0 else "StopStart"
            self._print_with_timestamp(f"\n=== Starting Block {i+1}/{self.blocks} ({method}) ===", total_start_time)
            self._run_block(method, duration=60.0, total_start_time=total_start_time)

        self.data_collector.stopDataCollection()
        total_elapsed = time.time() - total_start_time
        minutes = int(total_elapsed // 60)
        seconds = int(total_elapsed % 60)
        print(f"\nsystem identification run complete! Duration: {minutes:02d}:{seconds:02d}")
