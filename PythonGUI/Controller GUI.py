import customtkinter as ctk
import serial
import serial.tools.list_ports
import time
import threading
import struct
from queue import Queue
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from collections import deque

# Set the appearance mode and color theme
ctk.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class ReactionWheelGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Window setup
        self.title("Reaction Wheel Control System")
        self.geometry("1400x900")
        
        # Serial connection variables
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.command_queue = Queue()
        self.telemetry_queue = Queue()
        
        # Telemetry data storage
        self.rpm_history = deque(maxlen=100)
        self.torque_history = deque(maxlen=100)
        self.time_history = deque(maxlen=100)
        self.start_time = time.time()
        
        # Create main layout
        self.create_widgets()
        
        # Start update loop
        self.update_telemetry()
        
        # Make sure to close serial port on exit
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        # Create main container with grid layout
        self.main_container = ctk.CTkFrame(self)
        self.main_container.pack(fill="both", expand=True, padx=10, pady=10)
        self.main_container.grid_columnconfigure(1, weight=1)
        self.main_container.grid_rowconfigure(1, weight=1)
        
        # Left panel for controls
        self.control_panel = ctk.CTkFrame(self.main_container)
        self.control_panel.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)
        
        # Top panel for telemetry displays
        self.telemetry_panel = ctk.CTkFrame(self.main_container)
        self.telemetry_panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # Bottom panel for graphs
        self.graph_panel = ctk.CTkFrame(self.main_container)
        self.graph_panel.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        
        self.create_control_widgets()
        self.create_telemetry_widgets()
        self.create_graph_widgets()

    def create_control_widgets(self):
        # Title
        title_label = ctk.CTkLabel(self.control_panel, text="Reaction Wheel Control", 
                                   font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=10)
        
        # Connection frame
        self.connection_frame = ctk.CTkFrame(self.control_panel)
        self.connection_frame.pack(fill="x", padx=10, pady=10)
        
        # COM port selection
        ctk.CTkLabel(self.connection_frame, text="COM Port:").pack(side="left", padx=5)
        self.port_var = ctk.StringVar()
        self.port_dropdown = ctk.CTkOptionMenu(self.connection_frame, variable=self.port_var, 
                                               values=self.get_available_ports())
        self.port_dropdown.pack(side="left", padx=5)
        
        # Refresh ports button
        self.refresh_btn = ctk.CTkButton(self.connection_frame, text="Refresh", 
                                         command=self.refresh_ports, width=80)
        self.refresh_btn.pack(side="left", padx=5)
        
        # Connect button
        self.connect_btn = ctk.CTkButton(self.connection_frame, text="Connect", 
                                         command=self.toggle_connection, width=80)
        self.connect_btn.pack(side="left", padx=5)
        
        # Status indicator
        self.status_label = ctk.CTkLabel(self.connection_frame, text="●", 
                                         font=ctk.CTkFont(size=20),
                                         text_color="red")
        self.status_label.pack(side="left", padx=5)
        
        # Control mode selection
        mode_frame = ctk.CTkFrame(self.control_panel)
        mode_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(mode_frame, text="Control Mode:").pack(side="left", padx=5)
        self.mode_var = ctk.StringVar(value="speed")
        self.speed_mode_btn = ctk.CTkRadioButton(mode_frame, text="Speed", 
                                                 variable=self.mode_var, value="speed",
                                                 command=self.update_control_mode)
        self.speed_mode_btn.pack(side="left", padx=5)
        self.torque_mode_btn = ctk.CTkRadioButton(mode_frame, text="Torque", 
                                                  variable=self.mode_var, value="torque",
                                                  command=self.update_control_mode)
        self.torque_mode_btn.pack(side="left", padx=5)
        
        # Speed/Torque control
        self.control_frame = ctk.CTkFrame(self.control_panel)
        self.control_frame.pack(fill="x", padx=10, pady=10)
        
        self.control_label = ctk.CTkLabel(self.control_frame, text="Speed (RPM):")
        self.control_label.pack(pady=5)
        
        self.control_slider = ctk.CTkSlider(self.control_frame, from_=-15000, to=15000, 
                                            number_of_steps=300, width=300,
                                            command=self.on_slider_change)
        self.control_slider.set(0)
        self.control_slider.pack(pady=5)
        
        self.control_entry = ctk.CTkEntry(self.control_frame, width=100)
        self.control_entry.pack(pady=5)
        self.control_entry.bind("<Return>", self.on_entry_change)
        
        # Direction control
        dir_frame = ctk.CTkFrame(self.control_panel)
        dir_frame.pack(fill="x", padx=10, pady=10)
        
        self.direction_var = ctk.StringVar(value="cw")
        self.cw_btn = ctk.CTkRadioButton(dir_frame, text="CW", 
                                         variable=self.direction_var, value="cw")
        self.cw_btn.pack(side="left", padx=5)
        self.ccw_btn = ctk.CTkRadioButton(dir_frame, text="CCW", 
                                          variable=self.direction_var, value="ccw")
        self.ccw_btn.pack(side="left", padx=5)
        
        # Brake control
        brake_frame = ctk.CTkFrame(self.control_panel)
        brake_frame.pack(fill="x", padx=10, pady=10)
        
        self.brake_var = ctk.BooleanVar(value=False)
        self.brake_btn = ctk.CTkSwitch(brake_frame, text="Brake", 
                                       variable=self.brake_var,
                                       command=self.on_brake_change)
        self.brake_btn.pack(pady=5)
        
        # Emergency stop button
        self.estop_btn = ctk.CTkButton(self.control_panel, text="EMERGENCY STOP", 
                                       command=self.emergency_stop,
                                       fg_color="red", hover_color="darkred",
                                       font=ctk.CTkFont(size=16, weight="bold"),
                                       height=50)
        self.estop_btn.pack(fill="x", padx=10, pady=20)
        
        # PID parameters (collapsed by default)
        self.pid_frame = ctk.CTkFrame(self.control_panel)
        self.pid_frame.pack(fill="x", padx=10, pady=5)
        
        self.pid_toggle_btn = ctk.CTkButton(self.pid_frame, text="▼ PID Parameters", 
                                            command=self.toggle_pid_params,
                                            fg_color="transparent", hover_color="gray25",
                                            border_width=1, border_color="gray50")
        self.pid_toggle_btn.pack(fill="x", pady=5)
        
        self.pid_content_frame = ctk.CTkFrame(self.pid_frame)
        # Hidden by default
        
        # PID parameter entries
        for param in ["KP", "KI", "KD"]:
            frame = ctk.CTkFrame(self.pid_content_frame)
            frame.pack(fill="x", pady=2)
            ctk.CTkLabel(frame, text=f"{param}:", width=40).pack(side="left", padx=5)
            entry = ctk.CTkEntry(frame, width=80)
            entry.pack(side="left", padx=5)
            entry.insert(0, "1.0" if param == "KP" else "0.0")
            setattr(self, f"{param.lower()}_entry", entry)
        
        # Apply PID button
        self.apply_pid_btn = ctk.CTkButton(self.pid_content_frame, text="Apply PID", 
                                           command=self.apply_pid_params)
        self.apply_pid_btn.pack(pady=5)

    def create_telemetry_widgets(self):
        # Create a grid of telemetry displays
        self.telemetry_grid = ctk.CTkFrame(self.telemetry_panel)
        self.telemetry_grid.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Configure grid
        for i in range(3):
            self.telemetry_grid.grid_columnconfigure(i, weight=1)
        
        # Create telemetry displays
        self.telemetry_displays = {}
        
        telemetry_items = [
            ("Current RPM", "0 RPM", 0, 0),
            ("Target RPM", "0 RPM", 0, 1),
            ("Yaw Angle", "0.0°", 0, 2),
            ("Target Yaw", "0.0°", 1, 0),
            ("Wheel Torque", "0.0 mNm", 1, 1),
            ("Status", "Stopped", 1, 2),
            ("Direction", "CW", 2, 0),
            ("Mode", "Speed", 2, 1),
            ("Faults", "None", 2, 2)
        ]
        
        for label, initial_value, row, col in telemetry_items:
            frame = ctk.CTkFrame(self.telemetry_grid)
            frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
            
            title = ctk.CTkLabel(frame, text=label, font=ctk.CTkFont(size=14))
            title.pack(pady=(5, 0))
            
            value = ctk.CTkLabel(frame, text=initial_value, 
                                 font=ctk.CTkFont(size=18, weight="bold"))
            value.pack(pady=(0, 5))
            
            self.telemetry_displays[label] = value

    def create_graph_widgets(self):
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 4), dpi=100)
        self.fig.set_facecolor("#2b2b2b")
        
        # Create two subplots
        self.ax1 = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
        
        # Style the plots
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor("#2b2b2b")
            ax.grid(True, alpha=0.3)
            ax.tick_params(colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')
        
        # Set labels
        self.ax1.set_title("RPM over Time", color='white')
        self.ax1.set_xlabel("Time (s)", color='white')
        self.ax1.set_ylabel("RPM", color='white')
        
        self.ax2.set_title("Torque over Time", color='white')
        self.ax2.set_xlabel("Time (s)", color='white')
        self.ax2.set_ylabel("Torque (mNm)", color='white')
        
        # Create lines
        self.rpm_line, = self.ax1.plot([], [], 'cyan', linewidth=2)
        self.torque_line, = self.ax2.plot([], [], 'magenta', linewidth=2)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
        
        # Initial limits
        self.ax1.set_xlim(0, 20)
        self.ax1.set_ylim(-15000, 15000)
        self.ax2.set_xlim(0, 20)
        self.ax2.set_ylim(-100, 100)
        
        self.fig.tight_layout()

    def get_available_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["No ports found"]

    def refresh_ports(self):
        ports = self.get_available_ports()
        self.port_dropdown.configure(values=ports)
        if ports and ports[0] != "No ports found":
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if not self.serial_port:
            try:
                port = self.port_var.get()
                if port == "No ports found":
                    return
                
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.running = True
                self.serial_thread = threading.Thread(target=self.serial_worker)
                self.serial_thread.daemon = True
                self.serial_thread.start()
                
                self.connect_btn.configure(text="Disconnect")
                self.status_label.configure(text_color="green")
                
            except Exception as e:
                print(f"Connection error: {e}")
                self.status_label.configure(text_color="red")
        else:
            self.running = False
            if self.serial_thread:
                self.serial_thread.join(timeout=1)
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            
            self.connect_btn.configure(text="Connect")
            self.status_label.configure(text_color="red")

    def serial_worker(self):
        buffer = bytearray()
        while self.running:
            try:
                # Send commands from queue
                while not self.command_queue.empty():
                    cmd = self.command_queue.get()
                    self.serial_port.write(f"{cmd}\n".encode())
                
                # Read available data
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    
                    # Process buffer for telemetry data
                    while b"TELEMETRY:" in buffer:
                        start_idx = buffer.index(b"TELEMETRY:") + len(b"TELEMETRY:")
                        end_idx = buffer.find(b'\n', start_idx)
                        
                        if end_idx != -1:
                            telemetry_data = buffer[start_idx:end_idx]
                            self.process_telemetry(telemetry_data)
                            buffer = buffer[end_idx+1:]
                        else:
                            break
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Serial worker error: {e}")
                time.sleep(0.1)

    def process_telemetry(self, data):
        try:
            # Parse telemetry packet based on your structure
            if len(data) < 13:  # Minimum packet size
                return
            
            if data[0] == 0xFE and data[-1] == 0xFF:  # Check start/end markers
                # Add right after your check for valid packet (after the if statement checking for 0xFE and 0xFF)
                print("Raw telemetry packet:", " ".join([f"{b:02x}" for b in data]))

                idx = 1
                
                # Current yaw
                current_yaw = struct.unpack('<h', data[idx:idx+2])[0] / 100.0
                idx += 2
                
                # Target yaw
                target_yaw = struct.unpack('<h', data[idx:idx+2])[0] / 100.0
                idx += 2
                
                print(f"About to parse RPM at index {idx}, bytes: {data[idx]:02x} {data[idx+1]:02x}")
                # Wheel RPM
                wheel_rpm = struct.unpack('<h', data[idx:idx+2])[0]
                idx += 2
                print(f"Parsed RPM value: {wheel_rpm}")
                le_rpm = struct.unpack('<h', data[idx-2:idx])[0]  # Little-endian
                be_rpm = struct.unpack('>h', data[idx-2:idx])[0]  # Big-endian
                print(f"Alternative parsing - Little-endian: {le_rpm}, Big-endian: {be_rpm}")

                # Wheel torque
                wheel_torque = struct.unpack('<h', data[idx:idx+2])[0] / 100.0
                idx += 2
                # After parsing current_yaw:
                print(f"Current yaw value: {current_yaw}")

                # After parsing target_yaw:
                print(f"Target yaw value: {target_yaw}")

                # After parsing wheel_torque:
                print(f"Wheel torque value: {wheel_torque}")

                # Status and faults
                status = data[idx]
                idx += 1
                faults = data[idx]
                idx += 1
                
                # Manual mode
                manual_mode = data[idx] == 1
                
                # Update history for graphs
                current_time = time.time() - self.start_time
                self.time_history.append(current_time)
                self.rpm_history.append(wheel_rpm)
                self.torque_history.append(wheel_torque)
                
                # Put data in queue for GUI update
                self.telemetry_queue.put({
                    'current_yaw': current_yaw,
                    'target_yaw': target_yaw,
                    'wheel_rpm': wheel_rpm,
                    'wheel_torque': wheel_torque,
                    'status': status,
                    'faults': faults,
                    'manual_mode': manual_mode
                })
                
        except Exception as e:
            print(f"Telemetry processing error: {e}")

    def update_telemetry(self):
        # Update telemetry displays
        while not self.telemetry_queue.empty():
            data = self.telemetry_queue.get()
            
            # Update text displays
            self.telemetry_displays["Current RPM"].configure(text=f"{data['wheel_rpm']} RPM")
            self.telemetry_displays["Yaw Angle"].configure(text=f"{data['current_yaw']:.1f}°")
            self.telemetry_displays["Target Yaw"].configure(text=f"{data['target_yaw']:.1f}°")
            self.telemetry_displays["Wheel Torque"].configure(text=f"{data['wheel_torque']:.1f} mNm")
            
            # Status interpretation
            status_text = "Running" if data['status'] & 0x01 else "Stopped"
            self.telemetry_displays["Status"].configure(text=status_text)
            
            # Direction
            direction_text = "CCW" if data['status'] & 0x04 else "CW"
            self.telemetry_displays["Direction"].configure(text=direction_text)
            
            # Mode
            mode_text = "Torque" if data['status'] & 0x02 else "Speed"
            self.telemetry_displays["Mode"].configure(text=mode_text)
            
            # Faults
            fault_text = "FAULT" if data['faults'] else "None"
            self.telemetry_displays["Faults"].configure(
                text=fault_text,
                text_color="red" if data['faults'] else "green"
            )
        
        # Update graphs
        if self.time_history:
            self.rpm_line.set_data(list(self.time_history), list(self.rpm_history))
            self.torque_line.set_data(list(self.time_history), list(self.torque_history))
            
            # Adjust axes limits
            if self.time_history[-1] > 20:
                self.ax1.set_xlim(self.time_history[-1] - 20, self.time_history[-1])
                self.ax2.set_xlim(self.time_history[-1] - 20, self.time_history[-1])
            
            # Adjust y-limits if needed
            if self.rpm_history:
                rpm_min = min(self.rpm_history) - 1000
                rpm_max = max(self.rpm_history) + 1000
                if abs(rpm_max - rpm_min) < 100:
                    rpm_max += 100
                    rpm_min -= 100
                self.ax1.set_ylim(rpm_min, rpm_max)
            
            if self.torque_history:
                torque_min = min(self.torque_history) - 10
                torque_max = max(self.torque_history) + 10
                if abs(torque_max - torque_min) < 10:
                    torque_max += 10
                    torque_min -= 10
                self.ax2.set_ylim(torque_min, torque_max)
            
            self.canvas.draw()
        
        # Schedule next update
        self.after(50, self.update_telemetry)

    def update_control_mode(self):
        mode = self.mode_var.get()
        if mode == "speed":
            self.control_label.configure(text="Speed (RPM):")
            self.control_slider.configure(from_=-15000, to=15000)
        else:
            self.control_label.configure(text="Torque (mNm):")
            self.control_slider.configure(from_=-100, to=100)
        self.control_slider.set(0)

    def on_slider_change(self, value):
        self.control_entry.delete(0, "end")
        self.control_entry.insert(0, str(int(value)))
        if self.serial_port and self.running:
            if self.mode_var.get() == "speed":
                self.command_queue.put(f"S {int(value)}")
            else:
                self.command_queue.put(f"T {value}")

    def on_entry_change(self, event):
        try:
            value = float(self.control_entry.get())
            self.control_slider.set(value)
            if self.serial_port and self.running:
                if self.mode_var.get() == "speed":
                    self.command_queue.put(f"S {int(value)}")
                else:
                    self.command_queue.put(f"T {value}")
        except ValueError:
            pass

    def on_brake_change(self):
        if self.brake_var.get():
            self.command_queue.put("B ON")
        else:
            self.command_queue.put("B OFF")

    def emergency_stop(self):
        self.command_queue.put("S 0")
        self.command_queue.put("B ON")
        self.control_slider.set(0)
        self.control_entry.delete(0, "end")
        self.control_entry.insert(0, "0")

    def toggle_pid_params(self):
        if not hasattr(self, 'pid_expanded'):
            self.pid_expanded = False
        
        if self.pid_expanded:
            self.pid_content_frame.pack_forget()
            self.pid_toggle_btn.configure(text="▼ PID Parameters")
        else:
            self.pid_content_frame.pack(fill="x", padx=5, pady=5)
            self.pid_toggle_btn.configure(text="▲ PID Parameters")
        
        self.pid_expanded = not self.pid_expanded

    def apply_pid_params(self):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            self.command_queue.put(f"PID {kp} {ki} {kd}")
        except ValueError:
            pass

    def on_closing(self):
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=1)
        if self.serial_port:
            self.serial_port.close()
        self.destroy()

if __name__ == "__main__":
    app = ReactionWheelGUI()
    app.mainloop()
