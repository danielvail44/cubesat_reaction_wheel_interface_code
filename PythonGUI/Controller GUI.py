import tkinter as tk
import serial
import time
import serial.tools.list_ports

# ---------- Configuration ----------
SERIAL_PORT = "/dev/cu.usbserial-1420"      # Change this to your actual port (e.g., "/dev/ttyUSB0" for Linux)
BAUD_RATE = 115200        # Must match the baud rate on your ESP32/Teensy
# -----------------------------------

# Try to open serial port
#check gethub update
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
root.title("Control Panel")

# Styling
btn_font = ("Arial", 14)
btn_size = {"width": 10, "height": 2}

# Layout
tk.Label(root, text="Controls\nEach Send Overrides Previous Commands", font=("Arial", 16, "bold")).grid(row=0, column=2, pady=10)
tk.Label(root, text="Desired Angle\n (Degrees)", font=("Arial", 16, "bold")).grid(row=1, column=3, pady=10)
tk.Label(root, text="Desired Torque\n (-1 to 1)", font=("Arial", 16, "bold")).grid(row=1, column=2, pady=10)
tk.Label(root, text="Desired Speed\n (400 to 15000)", font=("Arial", 16, "bold")).grid(row=1, column=1, pady=10)

input_fieldA = tk.Entry(root, width=10)
input_fieldA.grid(row=2, column=3)

input_fieldT = tk.Entry(root, width=10)
input_fieldT.grid(row=2, column=2)

input_fieldS = tk.Entry(root, width=10)
input_fieldS.grid(row=2, column=1)

send_buttonA = tk.Button(root, text="Send", command=lambda: send_command("TARGET "+input_fieldA.get()))
send_buttonA.grid(row=3, column=3)

send_buttonT = tk.Button(root, text="Send", command=lambda: send_command("T "+input_fieldT.get()))
send_buttonT.grid(row=3, column=2)

send_buttonS = tk.Button(root, text="Send", command=lambda: send_command("S "+input_fieldS.get()))
send_buttonS.grid(row=3, column=1)

# Start GUI loop
root.mainloop()

# Close serial port on exit
if ser and ser.is_open:
    ser.close()
