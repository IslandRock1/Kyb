import tkinter as tk
import serial
import threading

# --- Configure serial port ---
# Change this to match your ESP32 COM port (Windows "COMx", Linux "/dev/ttyUSB0")
ser = serial.Serial('COM3', 115200, timeout=1)

# --- Tkinter GUI ---
root = tk.Tk()
root.title("ESP32 Control")

# Label to show response
response_label = tk.Label(root, text="ESP32 Response: ---")
response_label.pack(pady=10)

# Slider
slider0 = tk.Scale(root, from_=0, to=5000, orient="horizontal", label="Target Position Wrist", length=800)
slider0.pack(pady=10)

slider1 = tk.Scale(root, from_=-1000, to=1000, orient="horizontal", label="Target Position Shoulder", length=800)
slider1.pack(pady=10)

value0 = 0
value1 = 0

def send_value0(val):
    global value0
    value0 = float(val) * (-4.4)
    """Send slider value to ESP32"""
    msg = f"{value0},{value1}\n"   # match your ESP32 parser (comma-separated if more than one value)
    ser.write(msg.encode("utf-8"))

def send_value1(val):
    global value1
    value1 = float(val)
    """Send slider value to ESP32"""
    msg = f"{value0},{value1}\n"   # match your ESP32 parser (comma-separated if more than one value)
    ser.write(msg.encode("utf-8"))

slider0.config(command=send_value0)
slider1.config(command=send_value1)

def formatResponse(msg):
    if ("," not in msg): return ""
    v0, v1 = msg.split(",")
    if (v0 == ""): return ""
    if (v1 == ""): return ""
    v0 = float(v0) / 4096.0 * 360.0 / (-4.4)
    v1 = float(v1) / 4096.0 * 360.0

    global value0, value1

    d0 = v0 - value0
    d1 = v1 - value1

    return f"Shoulder: {v1:.2f} | Wrist: {v0:.2f}"

def read_serial():
    """Background thread to read responses from ESP32"""
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                response_label.config(text=f"ESP32 Response: {formatResponse(line)}")

# Run serial reader in a background thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Run GUI
root.mainloop()

# Close serial on exit
ser.close()
