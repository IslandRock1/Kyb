import tkinter as tk
import serial
import threading


class ESP32ControlApp:
    def __init__(self, port, baudrate):
        # --- Serial setup ---
        self.ser = serial.Serial(port, baudrate, timeout=1)

        # State variables
        self.value_wrist = 0.0
        self.value_shoulder = 0.0

        # --- Tkinter setup ---
        self.root = tk.Tk()
        self.root.title("ESP32 Control")

        # Response label
        self.response_label = tk.Label(self.root, text="ESP32 Response: ---")
        self.response_label.pack(pady=10)

        # Sliders
        self.slider_wrist = tk.Scale(
            self.root, from_=0, to=5000, orient="horizontal",
            label="Target Position Wrist", length=800,
            command=self.update_wrist
        )
        self.slider_wrist.pack(pady=10)

        self.slider_shoulder = tk.Scale(
            self.root, from_=-1000, to=1000, orient="horizontal",
            label="Target Position Shoulder", length=800,
            command=self.update_shoulder
        )
        self.slider_shoulder.pack(pady=10)

        # Start serial reader thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

        # Ensure serial port closes when window closes
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # --- Serial communication ---
    def send_values(self):
        """Send current slider values to ESP32"""
        msg = f"{self.value_wrist},{self.value_shoulder}\n"
        self.ser.write(msg.encode("utf-8"))

    def read_serial(self):
        """Background thread to read responses from ESP32"""
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    formatted = self.format_response(line)
                    if formatted:
                        self.response_label.config(text=f"ESP32 Response: {formatted}")

    # --- Slider callbacks ---
    def update_wrist(self, val):
        self.value_wrist = float(val)
        self.send_values()

    def update_shoulder(self, val):
        self.value_shoulder = float(val)
        self.send_values()

    # --- Helpers ---
    @staticmethod
    def format_response(msg):
        """Format incoming message (expected 'wrist,shoulder')"""
        if "," not in msg:
            return ""
        try:
            v0, v1 = msg.split(",")
            if not v0 or not v1:
                return ""
            v0, v1 = float(v0), float(v1)
            return f"Shoulder: {v1:.2f} | Wrist: {v0:.2f}"
        except ValueError:
            return ""

    def on_close(self):
        """Handle application exit"""
        self.running = False
        self.ser.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = ESP32ControlApp(port="COM3", baudrate=115200)
    app.run()
