import tkinter as tk
from tkinter import ttk
import serial
import threading
import re
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
from collections import deque

# --- Configuration ---
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
DATA_TIMEOUT_SECONDS = 1.0 # Timeout for serial read
MAX_GRAPH_POINTS = 100 # Number of data points for scrolling graphs

# --- Global Variables ---
ser = None
data_thread = None
stop_thread = False
latest_data = {
    "A": [0.0, 0.0, 0.0],  # Expected in mg
    "H": [0.0, 0.0, 0.0],  # Expected in g
    "P": 0.0,
    "Alt": 0.0, # Barometer Altitude
    "Q": [1.0, 0.0, 0.0, 0.0], # w, x, y, z
    "L": 0,
    "KF_Alt": 0.0, # Kalman Filter Altitude
    "KF_Vel": 0.0  # Kalman Filter Velocity
}
data_lock = threading.Lock()

# Data history for graphs
acc_g_x_hist = deque(maxlen=MAX_GRAPH_POINTS)
acc_g_y_hist = deque(maxlen=MAX_GRAPH_POINTS)
acc_g_z_hist = deque(maxlen=MAX_GRAPH_POINTS)
hg_acc_g_x_hist = deque(maxlen=MAX_GRAPH_POINTS)
hg_acc_g_y_hist = deque(maxlen=MAX_GRAPH_POINTS)
hg_acc_g_z_hist = deque(maxlen=MAX_GRAPH_POINTS)
graph_time_idx = deque(maxlen=MAX_GRAPH_POINTS)

baro_alt_hist = deque(maxlen=MAX_GRAPH_POINTS)
kf_alt_hist = deque(maxlen=MAX_GRAPH_POINTS)
kf_vel_hist = deque(maxlen=MAX_GRAPH_POINTS)

# --- 3D Cube Vertices (centered at origin) ---
# Cube vertices
v = np.array([
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [0.5, 0.5, 0.5],
    [-0.5, 0.5, 0.5]
])

# Cube faces (vertex indices)
faces = [
    [v[0], v[1], v[2], v[3]],
    [v[4], v[5], v[6], v[7]],
    [v[0], v[1], v[5], v[4]],
    [v[2], v[3], v[7], v[6]],
    [v[0], v[3], v[7], v[4]],
    [v[1], v[2], v[6], v[5]]
]

cube_face_colors = ['blue', 'red', 'green', 'yellow', 'cyan', 'magenta'] # Back, Front, Bottom, Top, Left, Right

# --- Serial Data Handling ---
def parse_data(line):
    """
    Parses a line of sensor data.
    Example: A:-20.5,-125.9,993.6|H:0.2,0.1,1.0|B:P102068,Alt0.1|Q:1.00,-0.06,0.01,0.02L:4ms
    """
    data = {}
    parts = line.split('|')

    for part in parts:
        if part.startswith("A:"):
            try:
                values = [float(x) for x in part[2:].split(',')]
                if len(values) == 3:
                    data["A"] = values # Store as mg
            except ValueError:
                pass # Ignore malformed A data
        elif part.startswith("H:"):
            try:
                values = [float(x) for x in part[2:].split(',')]
                if len(values) == 3:
                    data["H"] = values # Store as g
            except ValueError:
                pass
        elif part.startswith("B:"):
            try:
                baro_part = part[2:]
                if 'L:' in baro_part:
                    baro_part, l_part_from_b = baro_part.split('L:', 1)
                    parts.append(f"L:{l_part_from_b}")

                p_match = re.search(r"P(-?\d+\.?\d*)", baro_part)
                alt_match = re.search(r"Alt(-?\d+\.?\d*)", baro_part)
                if p_match: data["P"] = float(p_match.group(1))
                if alt_match: data["Alt"] = float(alt_match.group(1))
            except ValueError:
                pass
        elif part.startswith("Q:"):
            try:
                quat_part = part[2:]
                if 'L:' in quat_part:
                    quat_part, l_part_from_q = quat_part.split('L:', 1)
                    parts.append(f"L:{l_part_from_q}")
                values = [float(x) for x in quat_part.split(',')]
                if len(values) == 4: data["Q"] = values
            except ValueError:
                pass
        elif part.startswith("L:"):
            try:
                val = part[2:].replace("ms", "")
                data["L"] = int(val)
            except ValueError:
                pass
        elif part.startswith("KF:"):
            try:
                kf_part = part[3:] # Skip "KF:"
                alt_match = re.search(r"Alt(-?\d+\.?\d*)", kf_part)
                vel_match = re.search(r"Vel(-?\d+\.?\d*)", kf_part)
                if alt_match: data["KF_Alt"] = float(alt_match.group(1))
                if vel_match: data["KF_Vel"] = float(vel_match.group(1))
            except ValueError: pass
    return data

def read_from_port():
    global ser, latest_data, stop_thread

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=DATA_TIMEOUT_SECONDS)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        if 'status_label' in globals() and status_label: # Check if status_label is initialized
             status_label.config(text=f"Connected to {SERIAL_PORT}", foreground="green")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        if 'status_label' in globals() and status_label:
             status_label.config(text=f"Error: Could not open {SERIAL_PORT}", foreground="red")
        return

    while not stop_thread:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    parsed = parse_data(line)
                    with data_lock:
                        for key, value in parsed.items():
                            if key in latest_data: latest_data[key] = value
                            elif key == "P": latest_data["P"] = value
                            elif key == "Alt": latest_data["Alt"] = value
                else: # Timeout
                    pass
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                if 'status_label' in globals() and status_label: status_label.config(text="Serial disconnected.", foreground="red")
                if ser: ser.close()
                break
            except UnicodeDecodeError:
                print("Unicode decode error.")
            except Exception as e:
                print(f"Error in read_from_port: {e}")
        else:
            if not stop_thread:
                print(f"Attempting to reconnect to {SERIAL_PORT}...")
                if 'status_label' in globals() and status_label: status_label.config(text=f"Reconnecting to {SERIAL_PORT}...", foreground="orange")
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=DATA_TIMEOUT_SECONDS)
                    print(f"Reconnected to {SERIAL_PORT}.")
                    if 'status_label' in globals()and status_label: status_label.config(text=f"Connected to {SERIAL_PORT}", foreground="green")
                except serial.SerialException:
                    if 'status_label' in globals() and status_label: status_label.config(text=f"Reconnect failed. Retrying...", foreground="red")
                    time.sleep(2)
            else:
                break
    if ser and ser.is_open: ser.close()
    print("Serial reading thread stopped.")

# --- GUI Elements ---
root = tk.Tk()
root.title("Sensor Data Visualizer")

# Frame for text data
data_frame = ttk.LabelFrame(root, text="Sensor Readings")
data_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nswe")

labels = {}
label_texts = {
    "A_x": "Acc X (mg):", "A_y": "Acc Y (mg):", "A_z": "Acc Z (mg):",
    "H_x": "HG Acc X (g):", "H_y": "HG Acc Y (g):", "H_z": "HG Acc Z (g):",
    "P": "Pressure (Pa):", "Alt": "Baro Alt (m):",
    "Q_w": "Q w:", "Q_x": "Q x:", "Q_y": "Q y:", "Q_z": "Q z:",
    "L": "Loop Time (ms):",
    "KF_Alt": "KF Alt (m):", "KF_Vel": "KF Vel (m/s):"
}
for i, (key, text) in enumerate(label_texts.items()):
    ttk.Label(data_frame, text=text).grid(row=i, column=0, padx=5, pady=2, sticky="w")
    labels[key] = ttk.Label(data_frame, text="N/A")
    labels[key].grid(row=i, column=1, padx=5, pady=2, sticky="w")

# --- Matplotlib 3D Cube ---
fig_3d = plt.figure(figsize=(5, 4)) # Adjust size as needed
ax_3d = fig_3d.add_subplot(111, projection='3d')
canvas_3d = FigureCanvasTkAgg(fig_3d, master=root)
canvas_3d_widget = canvas_3d.get_tk_widget()
canvas_3d_widget.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

cube_collection = Poly3DCollection(faces, facecolors=cube_face_colors, linewidths=1, edgecolors='k', alpha=0.75)
ax_3d.add_collection3d(cube_collection)
ax_3d.set_xlabel('X'); ax_3d.set_ylabel('Y'); ax_3d.set_zlabel('Z')
ax_3d.set_xlim([-1, 1]); ax_3d.set_ylim([-1, 1]); ax_3d.set_zlim([-1, 1])
ax_3d.set_title("Orientation")
ax_3d.view_init(elev=20., azim=-35)

# --- Matplotlib 2D Accelerometer Graph ---
fig_accel, ax_accel = plt.subplots(figsize=(10, 3)) # Adjusted height
canvas_accel = FigureCanvasTkAgg(fig_accel, master=root)
canvas_accel_widget = canvas_accel.get_tk_widget()
canvas_accel_widget.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")

ax_accel.set_title("Accelerometer Data (Scrolling)")
ax_accel.set_xlabel("Sample Index")
ax_accel.set_ylabel("Acceleration (g)")
ax_accel.grid(True)
lines_accel = {
    "A_x": ax_accel.plot([], [], label="Acc X (g)", color="r")[0],
    "A_y": ax_accel.plot([], [], label="Acc Y (g)", color="g")[0],
    "A_z": ax_accel.plot([], [], label="Acc Z (g)", color="b")[0],
    "H_x": ax_accel.plot([], [], label="HG Acc X (g)", linestyle="--", color="c")[0],
    "H_y": ax_accel.plot([], [], label="HG Acc Y (g)", linestyle="--", color="m")[0],
    "H_z": ax_accel.plot([], [], label="HG Acc Z (g)", linestyle="--", color="y")[0],
}
ax_accel.legend(loc='upper left', fontsize='small')

# --- Matplotlib Altitude Comparison Graph ---
fig_alt, ax_alt = plt.subplots(figsize=(10, 3)) # Adjusted height
canvas_alt = FigureCanvasTkAgg(fig_alt, master=root)
canvas_alt_widget = canvas_alt.get_tk_widget()
canvas_alt_widget.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
ax_alt.set_title("Altitude Comparison (Scrolling)")
ax_alt.set_xlabel("Sample Index")
ax_alt.set_ylabel("Altitude (m)")
ax_alt.grid(True)
lines_alt = {
    "Baro_Alt": ax_alt.plot([], [], label="Barometer Alt (m)", color="purple")[0],
    "KF_Alt": ax_alt.plot([], [], label="Kalman Alt (m)", color="orange")[0],
}
ax_alt.legend(loc='upper left', fontsize='small')

# --- Matplotlib Kalman Velocity Graph ---
fig_vel, ax_vel = plt.subplots(figsize=(10, 3)) # Adjusted height
canvas_vel = FigureCanvasTkAgg(fig_vel, master=root)
canvas_vel_widget = canvas_vel.get_tk_widget()
canvas_vel_widget.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
ax_vel.set_title("Kalman Velocity (Scrolling)")
ax_vel.set_xlabel("Sample Index")
ax_vel.set_ylabel("Velocity (m/s)")
ax_vel.grid(True)
lines_vel = {
    "KF_Vel": ax_vel.plot([], [], label="Kalman Vel (m/s)", color="teal")[0],
}
ax_vel.legend(loc='upper left', fontsize='small')

# Status Label
status_label = ttk.Label(root, text="Initializing...", foreground="blue")
status_label.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

# --- GUI Update Function ---
def update_gui():
    global latest_data, graph_time_idx

    with data_lock:
        data_copy = latest_data.copy()

    # Update text labels
    for key in labels: # Iterate through all defined labels
        if key in data_copy:            
            if isinstance(data_copy[key], list) and len(data_copy[key]) > 0: # Handle list data (like A, H, Q)
                if key.startswith("Q"): # Quaternions
                     labels[key].config(text=f"{data_copy[key][label_texts[key].split(' ')[-1].count('.')]:.2f}") # hacky way to get index
                else: # Accel, HG Accel
                     labels[key].config(text=f"{data_copy[key][0]:.1f}") # Default to first element for simplicity if not split
            elif key == "P": labels[key].config(text=f"{data_copy[key]:.0f}")
            elif key in ["Alt", "KF_Alt", "KF_Vel"]: labels[key].config(text=f"{data_copy[key]:.2f}") # Ensure KF data is formatted
            else: labels[key].config(text=str(data_copy[key])) # General case
        elif key.startswith("A_") or key.startswith("H_") or key.startswith("Q_") : # Handle split labels like A_x, H_y, Q_w
            base_key = key.split('_')[0]
            idx = ['w','x','y','z'].index(key.split('_')[1]) if base_key == 'Q' else ['x','y','z'].index(key.split('_')[1])
            if base_key in data_copy and len(data_copy[base_key]) > idx:
                if base_key == 'Q': labels[key].config(text=f"{data_copy[base_key][idx]:.2f}")
                else: labels[key].config(text=f"{data_copy[base_key][idx]:.1f}")

    # Update 3D Cube
    try:
        q_w, q_x, q_y, q_z = data_copy['Q']
        r = R.from_quat([q_x, q_y, q_z, q_w])
        rotated_v = r.apply(v)
        new_faces = [[rotated_v[i] for i in face_indices] for face_indices in [[0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6], [0,3,7,4], [1,2,6,5]] ]
        cube_collection.set_verts(new_faces)
        canvas_3d.draw_idle()
    except Exception as e:
        print(f"Error updating 3D cube: {e}")

    # Update Graph Data
    current_time_idx = graph_time_idx[-1] + 1 if graph_time_idx else 0
    graph_time_idx.append(current_time_idx)

    acc_g_x_hist.append(data_copy['A'][0] / 1000.0) # Convert mg to g
    acc_g_y_hist.append(data_copy['A'][1] / 1000.0)
    acc_g_z_hist.append(data_copy['A'][2] / 1000.0)
    hg_acc_g_x_hist.append(data_copy['H'][0]) # Already in g
    hg_acc_g_y_hist.append(data_copy['H'][1])
    hg_acc_g_z_hist.append(data_copy['H'][2])
    baro_alt_hist.append(data_copy['Alt'])
    kf_alt_hist.append(data_copy['KF_Alt'])
    kf_vel_hist.append(data_copy['KF_Vel'])

    # Update plot lines
    lines_accel["A_x"].set_data(list(graph_time_idx), list(acc_g_x_hist))
    lines_accel["A_y"].set_data(list(graph_time_idx), list(acc_g_y_hist))
    lines_accel["A_z"].set_data(list(graph_time_idx), list(acc_g_z_hist))
    lines_accel["H_x"].set_data(list(graph_time_idx), list(hg_acc_g_x_hist))
    lines_accel["H_y"].set_data(list(graph_time_idx), list(hg_acc_g_y_hist))
    lines_accel["H_z"].set_data(list(graph_time_idx), list(hg_acc_g_z_hist))

    ax_accel.relim()
    ax_accel.autoscale_view(True, True, True) # Autoscale x and y axes
    if graph_time_idx: # Ensure x-axis scrolls
        ax_accel.set_xlim(graph_time_idx[0], graph_time_idx[-1])

    canvas_accel.draw_idle()
    
    # Update status label (moved after initial `ser` check in `read_from_port`)
    # The status label is now more directly updated by `read_from_port` for connection events.

    # Update Altitude Comparison graph
    lines_alt["Baro_Alt"].set_data(list(graph_time_idx), list(baro_alt_hist))
    lines_alt["KF_Alt"].set_data(list(graph_time_idx), list(kf_alt_hist))
    ax_alt.relim(); ax_alt.autoscale_view(True,True,True) # Autoscale Y for altitude
    if graph_time_idx: ax_alt.set_xlim(graph_time_idx[0], graph_time_idx[-1])
    canvas_alt.draw_idle()

    # Update Kalman Velocity graph
    lines_vel["KF_Vel"].set_data(list(graph_time_idx), list(kf_vel_hist))
    ax_vel.relim(); ax_vel.autoscale_view(True,True,True) # Autoscale Y for velocity
    if graph_time_idx: ax_vel.set_xlim(graph_time_idx[0], graph_time_idx[-1])
    canvas_vel.draw_idle()

    if not stop_thread:
        root.after(100, update_gui) # ~10Hz GUI refresh

# --- Start/Stop ---
def on_closing():
    global stop_thread, data_thread
    print("Closing application...")
    stop_thread = True
    if data_thread and data_thread.is_alive(): data_thread.join(timeout=2)
    if ser and ser.is_open: ser.close()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Configure grid weights
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1) # Adjust as needed, 1:1 for data & 3D plot side-by-side
root.rowconfigure(0, weight=0)  # Text data and 3D cube row (less weight)
root.rowconfigure(1, weight=1)  # Accel graph
root.rowconfigure(2, weight=1)  # Altitude graph
root.rowconfigure(3, weight=1)  # Velocity graph
root.rowconfigure(4, weight=0)  # Status label (less weight)

# Start serial reading
data_thread = threading.Thread(target=read_from_port, daemon=True)
data_thread.start()

root.after(100, update_gui)
root.mainloop()
print("Application closed.") 