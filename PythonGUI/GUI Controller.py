import tkinter as tk
import serial
import serial.tools.list_ports

# ---------- Configuration ----------
SERIAL_PORT = "COM5"      # Change this to your actual port (e.g., "/dev/ttyUSB0" for Linux)
BAUD_RATE = 9600        # Must match the baud rate on your ESP32/Teensy
# -----------------------------------

# Try to open serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Function to send a command
def send_command(cmd):
    if ser and ser.is_open:
        ser.write(f"{cmd}\n".encode())
        print(f"Sent: {cmd}")
    else:
        print("Serial port is not open.")

# GUI setup
root = tk.Tk()
root.title("Directional Controller")

# Styling
btn_font = ("Arial", 14)
btn_size = {"width": 10, "height": 2}

# Layout
tk.Label(root, text="Directional Controller", font=("Arial", 16, "bold")).grid(row=0, column=1, pady=10)

btn_up = tk.Button(root, text="↑", font=btn_font, **btn_size, command=lambda: send_command("up"))
btn_up.grid(row=1, column=1)

btn_left = tk.Button(root, text="←", font=btn_font, **btn_size, command=lambda: send_command("left"))
btn_left.grid(row=2, column=0)

btn_stop = tk.Button(root, text="■", font=btn_font, fg="red", **btn_size, command=lambda: send_command("stop"))
btn_stop.grid(row=2, column=1)

btn_right = tk.Button(root, text="→", font=btn_font, **btn_size, command=lambda: send_command("right"))
btn_right.grid(row=2, column=2)

btn_down = tk.Button(root, text="↓", font=btn_font, **btn_size, command=lambda: send_command("down"))
btn_down.grid(row=3, column=1)

# Exit button
btn_exit = tk.Button(root, text="Exit", font=("Arial", 12), command=root.quit)
btn_exit.grid(row=4, column=1, pady=10)

# Start GUI loop
root.mainloop()

# Close serial port on exit
if ser and ser.is_open:
    ser.close()
