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

# --- Configuration ---
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
DATA_TIMEOUT_SECONDS = 1.0 # Timeout for serial read

# --- Global Variables ---
ser = None
data_thread = None
stop_thread = False
latest_data = {
    "A": [0.0, 0.0, 0.0],
    "H": [0.0, 0.0, 0.0],
    "P": 0.0,
    "Alt": 0.0,
    "Q": [1.0, 0.0, 0.0, 0.0], # w, x, y, z
    "L": 0
}
data_lock = threading.Lock()

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
                    data["A"] = values
            except ValueError:
                pass # Ignore malformed A data
        elif part.startswith("H:"):
            try:
                values = [float(x) for x in part[2:].split(',')]
                if len(values) == 3:
                    data["H"] = values
            except ValueError:
                pass
        elif part.startswith("B:"):
            try:
                # B:P102068,Alt0.1 or B:P102068,Alt0.1L:4ms (if L is attached)
                baro_part = part[2:]
                # Separate L if it's concatenated
                if 'L:' in baro_part:
                    baro_part, l_part_from_b = baro_part.split('L:', 1)
                    parts.append(f"L:{l_part_from_b}") # Add L part back for parsing

                p_match = re.search(r"P(-?\d+\.?\d*)", baro_part)
                alt_match = re.search(r"Alt(-?\d+\.?\d*)", baro_part)
                if p_match:
                    data["P"] = float(p_match.group(1))
                if alt_match:
                    data["Alt"] = float(alt_match.group(1))
            except ValueError:
                pass
        elif part.startswith("Q:"):
            try:
                 # Q:1.00,-0.06,0.01,0.02 or Q:1.00,-0.06,0.01,0.02L:4ms
                quat_part = part[2:]
                if 'L:' in quat_part:
                    quat_part, l_part_from_q = quat_part.split('L:', 1)
                    parts.append(f"L:{l_part_from_q}")

                values = [float(x) for x in quat_part.split(',')]
                if len(values) == 4:
                    data["Q"] = values # Assuming q0, q1, q2, q3 (w, x, y, z)
            except ValueError:
                pass
        elif part.startswith("L:"):
            try:
                val = part[2:].replace("ms", "")
                data["L"] = int(val)
            except ValueError:
                pass
    return data

def read_from_port():
    global ser, latest_data, stop_thread

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=DATA_TIMEOUT_SECONDS)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        # Update GUI to show error
        if 'status_label' in globals():
             status_label.config(text=f"Error: Could not open {SERIAL_PORT}", foreground="red")
        return

    while not stop_thread:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # print(f"Raw: {line}") # For debugging
                    parsed = parse_data(line)
                    # print(f"Parsed: {parsed}") # For debugging
                    with data_lock:
                        # Update only fields present in parsed data
                        for key, value in parsed.items():
                            if key in latest_data: # Ensure we only update known keys
                                latest_data[key] = value
                            elif key == "P": # Handle P and Alt separately if needed
                                latest_data["P"] = value
                            elif key == "Alt":
                                latest_data["Alt"] = value
                else: # Timeout occurred
                    # print("Serial read timeout") # Optional: indicate timeout
                    pass
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                if 'status_label' in globals():
                    status_label.config(text="Serial disconnected.", foreground="red")
                ser.close() # Attempt to close on error
                break # Exit thread on critical serial error
            except UnicodeDecodeError:
                print("Unicode decode error. Skipping line.") # Skip corrupted data
            except Exception as e:
                print(f"Unexpected error in read_from_port: {e}") # Catch other errors
        else:
            # Attempt to reconnect if port was closed or not opened initially
            if not stop_thread:
                print(f"Attempting to reconnect to {SERIAL_PORT}...")
                if 'status_label' in globals():
                    status_label.config(text=f"Attempting to reconnect to {SERIAL_PORT}...", foreground="orange")
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=DATA_TIMEOUT_SECONDS)
                    print(f"Reconnected to {SERIAL_PORT}.")
                    if 'status_label' in globals():
                        status_label.config(text=f"Connected to {SERIAL_PORT}", foreground="green")
                except serial.SerialException:
                    if 'status_label' in globals():
                        status_label.config(text=f"Reconnect failed. Will retry...", foreground="red")
                    time.sleep(2) # Wait before retrying
            else:
                break
    if ser and ser.is_open:
        ser.close()
    print("Serial reading thread stopped.")

# --- GUI Elements ---
root = tk.Tk()
root.title("Sensor Data Visualizer")

# Frame for text data
data_frame = ttk.LabelFrame(root, text="Sensor Readings")
data_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

# Labels for data
labels = {}
label_texts = {
    "A_x": "Acc X (mg):", "A_y": "Acc Y (mg):", "A_z": "Acc Z (mg):",
    "H_x": "HG Acc X (g):", "H_y": "HG Acc Y (g):", "H_z": "HG Acc Z (g):",
    "P": "Pressure (Pa):", "Alt": "Altitude (m):",
    "Q_w": "Q w:", "Q_x": "Q x:", "Q_y": "Q y:", "Q_z": "Q z:",
    "L": "Loop Time (ms):"
}
row_idx = 0
for key, text in label_texts.items():
    ttk.Label(data_frame, text=text).grid(row=row_idx, column=0, padx=5, pady=2, sticky="w")
    labels[key] = ttk.Label(data_frame, text="N/A")
    labels[key].grid(row=row_idx, column=1, padx=5, pady=2, sticky="w")
    row_idx += 1

# Status Label
status_label = ttk.Label(root, text="Connecting...", foreground="orange")
status_label.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")


# --- Matplotlib 3D Cube ---
fig_3d = plt.figure(figsize=(5, 4))
ax_3d = fig_3d.add_subplot(111, projection='3d')
canvas_3d = FigureCanvasTkAgg(fig_3d, master=root)
canvas_3d_widget = canvas_3d.get_tk_widget()
canvas_3d_widget.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew") # Span rows for better layout

cube_collection = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25)
ax_3d.add_collection3d(cube_collection)

ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')
ax_3d.set_xlim([-1, 1])
ax_3d.set_ylim([-1, 1])
ax_3d.set_zlim([-1, 1])
ax_3d.set_title("Orientation (Quaternion)")
ax_3d.view_init(elev=20., azim=-35) # Initial view angle

# --- GUI Update Function ---
def update_gui():
    global latest_data

    with data_lock:
        data_copy = latest_data.copy() # Work with a copy

    labels["A_x"].config(text=f"{data_copy['A'][0]:.1f}")
    labels["A_y"].config(text=f"{data_copy['A'][1]:.1f}")
    labels["A_z"].config(text=f"{data_copy['A'][2]:.1f}")

    labels["H_x"].config(text=f"{data_copy['H'][0]:.1f}")
    labels["H_y"].config(text=f"{data_copy['H'][1]:.1f}")
    labels["H_z"].config(text=f"{data_copy['H'][2]:.1f}")

    labels["P"].config(text=f"{data_copy['P']:.0f}")
    labels["Alt"].config(text=f"{data_copy['Alt']:.1f}")

    q_w, q_x, q_y, q_z = data_copy['Q']
    labels["Q_w"].config(text=f"{q_w:.2f}")
    labels["Q_x"].config(text=f"{q_x:.2f}")
    labels["Q_y"].config(text=f"{q_y:.2f}")
    labels["Q_z"].config(text=f"{q_z:.2f}")

    labels["L"].config(text=f"{data_copy['L']}")

    # Update 3D Cube
    # Quaternion: w, x, y, z. Scipy Rotation expects x, y, z, w
    try:
        # Scipy's Rotation takes (x, y, z, w)
        r = R.from_quat([q_x, q_y, q_z, q_w])
        rotated_v = r.apply(v) # v are original cube vertices

        # Reconstruct faces for Poly3DCollection
        new_faces = [
            [rotated_v[0], rotated_v[1], rotated_v[2], rotated_v[3]],
            [rotated_v[4], rotated_v[5], rotated_v[6], rotated_v[7]],
            [rotated_v[0], rotated_v[1], rotated_v[5], rotated_v[4]],
            [rotated_v[2], rotated_v[3], rotated_v[7], rotated_v[6]],
            [rotated_v[0], rotated_v[3], rotated_v[7], rotated_v[4]],
            [rotated_v[1], rotated_v[2], rotated_v[6], rotated_v[5]]
        ]
        cube_collection.set_verts(new_faces)
        canvas_3d.draw_idle() # Redraw the 3D canvas
    except Exception as e:
        print(f"Error updating 3D cube: {e}")


    # Check serial connection status (simple check)
    if ser and ser.is_open:
        if status_label.cget("text") != f"Connected to {SERIAL_PORT}":
             status_label.config(text=f"Connected to {SERIAL_PORT}", foreground="green")
    elif not stop_thread and (not ser or not ser.is_open) and status_label.cget("text").startswith("Attempting to reconnect"):
        pass # Keep "Attempting to reconnect..."
    elif not stop_thread and (not ser or not ser.is_open) and not status_label.cget("text").startswith("Error: Could not open"):
        if status_label.cget("text") != "Serial disconnected. Will retry...":
             status_label.config(text="Serial disconnected. Will retry...", foreground="red")


    if not stop_thread:
        root.after(100, update_gui) # Schedule next update (100ms for ~10Hz GUI update)

# --- Start/Stop ---
def on_closing():
    global stop_thread, data_thread
    print("Closing application...")
    stop_thread = True
    if data_thread and data_thread.is_alive():
        data_thread.join(timeout=2) # Wait for thread to finish
    if ser and ser.is_open:
        ser.close()
    root.quit() # Quit Tkinter main loop
    root.destroy() # Destroy the window

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start serial reading in a separate thread
data_thread = threading.Thread(target=read_from_port, daemon=True)
data_thread.start()

# Start GUI update loop
root.after(100, update_gui)

# Configure grid weights for resizing
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=3) # Give more weight to the 3D plot
root.rowconfigure(0, weight=1)


root.mainloop()

print("Application closed.") 