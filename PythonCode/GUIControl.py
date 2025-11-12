import tkinter as tk
import serial
import threading
from dataclasses import dataclass
from time import  perf_counter
import numpy as np

from PythonCode.utils.SensorHelpers import getForceVector, getMassVector, SensorToWorldFromSlides, getCLQ, compute_wrench
from PythonCode.utils.CalculateCenterOfMass import getCenterOfMass

def formatForceLabel(x):
    out = round(float(x), 2)
    return f" {out: .2f}"

@dataclass
class SerialData:
    position0: float
    position1: float
    power0: int
    power1: int
    positionMode0: bool
    positionMode1: bool

def rgb_to_hex(r, g, b):
    return f"#{r:02x}{g:02x}{b:02x}"

class ESP32ControlApp:
    def __init__(self, port, baudrate, portSensor, baudrateSensor):
        # --- Serial setup ---

        self.ser = None
        if (port is not None):
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.1)
                print("Got the serial port.")
            except Exception as e:
                print(f"Error opening serial port: {e}")
                pass

        self.serSensor = None
        if (portSensor is not None):
            try:
                self.serSensor = serial.Serial(portSensor, baudrateSensor, timeout=0.1)
            except Exception as e:
                pass
        self.running = True

        self.responses = []

        # State variables
        self.serialData = SerialData(0, 0, 0, 0, False, False)

        # --- Tkinter setup ---
        self.root = tk.Tk()
        self.root.title("ESP32 Control")
        self.root.geometry("800x500")

        # ESP32 response label
        self.response_label = tk.Label(self.root, text="ESP32 Response: ---", font=("Consolas", 10))
        self.response_label.pack(pady=10)

        self.forceLabel = tk.Label(self.root, text="Force: ---", font=("Consolas", 10))
        self.forceLabel.pack(pady=10)

        self.filteredLabel = tk.Label(self.root, text="Filtered: ---", font=("Consolas", 10))
        self.filteredLabel.pack(pady=10)

        # Create controls
        self.create_joint_controls("Wrist", 0)
        self.create_joint_controls("Shoulder", 1)

        self.record_button = tk.Button(self.root, text="Record", fg="white", bg="red", font=("Arial", 14, "bold"),
                                command=self.activate_recording)
        self.record_button.pack(pady=10)
        self.recording_time = perf_counter() - 5.0

        self.resetAngles = tk.Button(self.root, text = "Set initial angles.", fg = "white", bg = "black", font=("Arial", 14, "bold"),command=self.resetAnglesCommand)
        self.resetAngles.pack(pady=10)
        self.doResetAngles = False

        # Start background thread for serial reading
        threading.Thread(target=self.read_serial, daemon=True).start()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.updateButtonColor()

        self.currentForceValues = []
        self.currentAngleValues = []
        self.timeToUpdateAngles = False
        self.forceLabelValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.alpha = 0.01
        setattr(self, f"timeToUpdateAngles", self.timeToUpdateAngles)

        self.updateForceLabel()

    def updateButtonColor(self):

        remainingTime = 5 + self.recording_time - perf_counter()
        if (remainingTime < 0):
            self.record_button.config(bg = rgb_to_hex(0, 255, 0))
        else:
            self.record_button.config(bg = rgb_to_hex(int(255.0 * remainingTime / 5.0), 0, 0))

        self.root.after(10, self.updateButtonColor)

    # -----------------------------
    # UI Creation
    # -----------------------------
    def create_joint_controls(self, name: str, index: int):
        """Create a position and power control section for a joint"""
        frame = tk.LabelFrame(self.root, text=name, padx=10, pady=10)
        frame.pack(padx=10, pady=10, fill="x")

        # --- Position control ---
        pos_frame = tk.Frame(frame)
        pos_frame.pack(fill="x", pady=5)

        pos_slider = tk.Scale(
            pos_frame, from_=-360, to=360, orient="horizontal",
            label=f"{name} Target Position (°)", length=600,
            command=lambda val, idx=index: self.update_position(idx, val)
        )
        pos_slider.pack(side="left", padx=5)

        pos_entry = tk.Entry(pos_frame, width=8)
        pos_entry.insert(0, "0")
        pos_entry.pack(side="left", padx=5)
        pos_entry.bind("<Return>", lambda e, idx=index: self.set_position_from_entry(idx, e))

        # --- Power control ---
        power_frame = tk.Frame(frame)
        power_frame.pack(fill="x", pady=5)

        power_slider = tk.Scale(
            power_frame, from_=-255, to=255, orient="horizontal",
            label=f"{name} Power", length=600,
            command=lambda val, idx=index: self.update_power(idx, val)
        )
        power_slider.pack(side="left", padx=5)

        power_entry = tk.Entry(power_frame, width=8)
        power_entry.insert(0, "0")
        power_entry.pack(side="left", padx=5)
        power_entry.bind("<Return>", lambda e, idx=index: self.set_power_from_entry(idx, e))

        # Store widgets
        setattr(self, f"slider_pos_{index}", pos_slider)
        setattr(self, f"entry_pos_{index}", pos_entry)
        setattr(self, f"slider_power_{index}", power_slider)
        setattr(self, f"entry_power_{index}", power_entry)

    # -----------------------------
    # Serial Communication
    # -----------------------------
    def send_values(self):
        """Send current values to ESP32"""
        msg = (
            f"{self.serialData.position0},{self.serialData.position1},"
            f"{self.serialData.power0},{self.serialData.power1},"
            f"{int(self.serialData.positionMode0)},{int(self.serialData.positionMode1)},{int(self.doResetAngles)}\n"
        )
        self.doResetAngles = False

        if (self.ser is not None):
            self.ser.write(msg.encode("utf-8"))
            # print(f"Sent {msg}")

    def updateForceLabel(self):
        mass, COG = getCenterOfMass()

        if (len(self.currentForceValues) != 0) and (len(self.currentAngleValues) != 0):

            C, L, Q = getCLQ()

            currentForceValues = self.currentForceValues.copy()
            tmp = currentForceValues[5]
            currentForceValues[5] = currentForceValues[7]
            currentForceValues[7] = tmp
            West = compute_wrench(C, L, Q, currentForceValues)

            out = [float(formatForceLabel(x)) for x in West]
            newOut = [self.forceLabelValues[i] * (1 - self.alpha) + out[i] * self.alpha for i in range(6)]
            self.forceLabelValues = newOut
            self.forceLabel.config(text=f"W_est = {"".join([formatForceLabel(x) for x in West])}")


            # print(f"Shoulder angle: {self.currentAngleValues[1]} | Wrist angle: {self.currentAngleValues[0]}")
            Rsw = SensorToWorldFromSlides(self.currentAngleValues[1], self.currentAngleValues[0])

            forceVector = getForceVector(mass, Rsw).flatten()
            massVector = getMassVector(COG, forceVector).flatten()
            print(f"Mass: {mass} | Rsw: {Rsw} | forceVector: {forceVector}")
            self.filteredLabel.config(text = f"{forceVector[0,0]},{forceVector[0,1]},{forceVector[0,2]},{massVector[0]},{massVector[1]},{massVector[2]}")

            # self.filteredLabel.config(text = f"Filtered: {"".join([formatForceLabel(x) for x in newOut])}")

        self.root.after(100, self.updateForceLabel)

    def read_serial(self):
        """Read responses from ESP32 in background"""

        mass, COG = getCenterOfMass()

        while self.running:
            if (self.serSensor is not None):
                try:
                    if self.serSensor.in_waiting:
                        line = self.serSensor.readline().decode(errors="ignore").strip()
                        self.timeToUpdateAngles = True

                        readings: list[str] = line.split(" ")[3:]
                        num = readings.count("")
                        for _ in range(num): readings.remove("")

                        self.currentForceValues = [int(x) for x in readings[-8:]]

                        print(line)
                        if (perf_counter() - self.recording_time) < 5.0:
                            self.responses.append(f"{perf_counter()},FORCE,{line} {mass} {COG[0,0]} {COG[1,0]} {COG[2,0]}\n")
                except Exception as e:
                    print(e)

            if (self.ser is not None):
                try:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode(errors="ignore").strip()

                        if (self.timeToUpdateAngles):
                            self.timeToUpdateAngles = False
                            values = [float(x) for x in line.split(",")][0:4]
                            values[0] = np.deg2rad(values[0])  # Konverter wrist til radianer
                            values[1] = np.deg2rad(values[1])  # Konverter shoulder til radianer
                            self.currentAngleValues = values

                        if (perf_counter() - self.recording_time) < 5.0:
                            self.responses.append(f"{perf_counter()},ROBOT,{line}\n")
                        if line:
                            formatted = self.format_response(line)
                            if formatted:
                                self.root.after(0, lambda: self.response_label.config(text=f"ESP32 Response: {formatted}"))
                except Exception as e:
                    print(e)

    # -----------------------------
    # Slider & Entry Callbacks
    # -----------------------------
    def update_position(self, index, val):
        val = float(val)
        setattr(self.serialData, f"position{index}", val)
        setattr(self.serialData, f"positionMode{index}", True)

        entry = getattr(self, f"entry_pos_{index}")
        entry.delete(0, tk.END)
        entry.insert(0, str(int(val)))
        self.send_values()

    def update_power(self, index, val):
        val = int(val)
        setattr(self.serialData, f"power{index}", val)
        setattr(self.serialData, f"positionMode{index}", False)

        entry = getattr(self, f"entry_power_{index}")
        entry.delete(0, tk.END)
        entry.insert(0, str(val))
        self.send_values()

    def set_position_from_entry(self, index, event):
        try:
            val = float(getattr(self, f"entry_pos_{index}").get())
            getattr(self, f"slider_pos_{index}").set(val)
        except ValueError:
            pass

    def set_power_from_entry(self, index, event):
        try:
            val = int(getattr(self, f"entry_power_{index}").get())
            getattr(self, f"slider_power_{index}").set(val)
        except ValueError:
            pass

    def activate_recording(self):
        self.recording_time = perf_counter()

    def resetAnglesCommand(self):
        self.doResetAngles = True

    # -----------------------------
    # Helpers
    # -----------------------------
    @staticmethod
    def format_response(msg: str):
        try:
            v0, v1, v2, v3, v4, v5 = msg.split(",")
            v0, v1 = float(v0), float(v1)
            v2, v3 = float(v2) * 12.0 / 255, float(v3) * 12.0 / 255
            return f"Wrist: {v0:.2f}° ({v2:.2f}V) | Shoulder: {v1:.2f}° ({v3:.2f}V)"
        except ValueError:
            return ""

    def on_close(self):
        """Close the app cleanly"""
        self.running = False
        if (self.ser is not None) and self.ser.is_open:
            self.ser.close()
        if (self.serSensor is not None) and self.serSensor.is_open:
            self.serSensor.close()
        self.root.destroy()

        if (len(self.responses) > 0):
            print("Saving to file..")
            with open("PythonCode/DATA/response.txt", "a") as f:
                f.writelines(self.responses)
            print("Finished saving to file.")
        else:
            print("No results to save.")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    print()
    print()
    app = ESP32ControlApp(port="COM5", baudrate=115200, portSensor="COM7", baudrateSensor=115200)
    app.run()
