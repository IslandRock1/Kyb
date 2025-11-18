import csv
import time
import threading

from robotController import RobotController

class SystemIdentificationDataCollector:
    def __init__(self, robot: "RobotController", joint: str, filename: str):
        self.robot = robot
        self.joint = joint.lower()
        if self.joint not in ["shoulder", "wrist"]:
            raise ValueError("joint must be shoulder or wrist")
        
        self.filename = filename
        self.collecting = False
        self._data = []
        self._thread = None
        self._start_time = None
        self.movement_description = ""

    def _collect_loop(self):
        while self.collecting:
            timestamp_ms = int((time.time() - self._start_time) * 1000)
            
            if self.joint == "shoulder":
                pos = self.robot.getShoulderPosition()
                gain = self.robot._shoulder_gain
            else:
                pos = self.robot.getWristPosition()
                gain = self.robot._wrist_gain

            self._data.append([timestamp_ms, pos, gain, self.movement_description])
            time.sleep(0.01)

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
        print(f"Started data collection for {self.joint} joint.")

    def stopDataCollection(self):
        if not self.collecting:
            print("Data collection not running.")
            return
        self.collecting = False
        self._thread.join()

        with open(self.filename, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "position", "gain", "movement_description"])
            writer.writerows(self._data)

        print(f"Data collection stopped. Saved {len(self._data)} samples to {self.filename}.")

    def setMovementDescription(self, description: str):
        self.movement_description = description
