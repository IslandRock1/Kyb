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
slider = tk.Scale(root, from_=-1000, to=1000, orient="horizontal", label="Target Position")
slider.pack(pady=10)

def send_value(val):
    """Send slider value to ESP32"""
    msg = f"{val}\n"   # match your ESP32 parser (comma-separated if more than one value)
    ser.write(msg.encode("utf-8"))

slider.config(command=send_value)

def read_serial():
    """Background thread to read responses from ESP32"""
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                response_label.config(text=f"ESP32 Response: {line}")

# Run serial reader in a background thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Run GUI
root.mainloop()

# Close serial on exit
ser.close()
