import tkinter as tk
import serial
import threading
from dataclasses import dataclass
from time import  perf_counter

@dataclass
class SerialData:
    position0: float
    position1: float
    power0: int
    power1: int
    positionMode0: bool
    positionMode1: bool


class ESP32ControlApp:
    def __init__(self, port, baudrate, portSensor, baudrateSensor):
        # --- Serial setup ---
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.serSensor = serial.Serial(portSensor, baudrateSensor, timeout=0.1)
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

        # Create controls
        self.create_joint_controls("Wrist", 0)
        self.create_joint_controls("Shoulder", 1)

        # Start background thread for serial reading
        threading.Thread(target=self.read_serial, daemon=True).start()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

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
            f"{int(self.serialData.positionMode0)},{int(self.serialData.positionMode1)}\n"
        )
        self.ser.write(msg.encode("utf-8"))
        #print(f"Sent {msg}")

    def read_serial(self):
        """Read responses from ESP32 in background"""

        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                self.responses.append(f"{perf_counter()},ROBOT,{line}\n")
                if line:
                    formatted = self.format_response(line)
                    if formatted:
                        self.root.after(0, lambda: self.response_label.config(text=f"ESP32 Response: {formatted}"))

            try:
                if self.serSensor.in_waiting:
                    line = self.serSensor.readline().decode(errors="ignore").strip()
                    print(line)
                    self.responses.append(f"{perf_counter()},FORCE,{line}\n")
            except Exception as e:
                print(e)
                self.running = False

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
        if self.ser.is_open:
            self.ser.close()
        if self.serSensor.is_open:
            self.serSensor.close()
        self.root.destroy()

        print("Saving to file..")
        with open("response.txt", "w") as f:
            f.writelines(self.responses)
        print("Finished saving to file.")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    print()
    print()
    app = ESP32ControlApp(port="COM3", baudrate=115200, portSensor="COM6", baudrateSensor=115200)
    app.run()
