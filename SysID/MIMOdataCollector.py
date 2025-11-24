import csv
import time
import threading
from robotController import RobotController

class MIMOSystemIdentificationDataCollector:
    def __init__(self, robot: "RobotController", filename: str):
        self.robot = robot
        self.filename = filename
        self.collecting = False
        self._data = []
        self._thread = None
        self._start_time = None
        self.description = ""

    def _collect_loop(self):
        sampling_interval = 0.01
        next_sample_time = self._start_time

        while self.collecting:
            now = time.time()
            timestamp_s = now - self._start_time

            sh_pos = self.robot.getShoulderPosition()
            wr_pos = self.robot.getWristPosition()

            sh_gain = self.robot.getShoulderGain()
            wr_gain = self.robot.getWristGain()

            self._data.append([
                timestamp_s, sh_pos, wr_pos, sh_gain, wr_gain, self.description
            ])

            next_sample_time += sampling_interval
            sleep_time = max(0, next_sample_time - time.time())
            time.sleep(sleep_time)

    def startDataCollection(self):
        if self.collecting:
            print("Data collection already running.")
            return
        self.movement_description = "No Movement"
        self._data = []
        self._start_time = time.time()
        self.collecting = True

        self._thread = threading.Thread(target=self._collect_loop, daemon=True)
        self._thread.start()

    def stopDataCollection(self):
        if not self.collecting:
            print("Data collection not running.")
            return

        self.collecting = False
        self._thread.join()

        with open(self.filename, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "shoulder_position", "wrist_position",
                "shoulder_gain", "wrist_gain", "movement_description"
            ])
            writer.writerows(self._data)

        print(f"Saved {len(self._data)} samples → {self.filename}")

    def setDescription(self, desc: str):
        self.description = desc