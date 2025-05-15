import serial
import tkinter as tk

class SensorDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensor Dashboard")
        self.root.geometry("400x300")

        self.data_pattern = r'A:(-?\d+\.\d+),(-?\d+\.\d+),(\d+\.\d+)\|H:(\d+\.\d+),(\d+\.\d+),(\d+\.\d+)\|B:(\w+),(\w+)\|Q:(\d+\.\d+),(-?\d+\.\d+),(\d+\.\d+),(\d+\.\d+)\|L:(\d+ms)'

        self.create_widgets()

    def create_widgets(self):
        self.label = tk.Label(self.root, text="Input Data String:")
        self.label.pack(pady=10)

        self.entry = tk.Entry(self.root)
        self.entry.pack(pady=10)

        self.update_button = tk.Button(self.root, text="Update Data", command=self.update_from_entry)
        self.update_button.pack(pady=10)

        self.data_label = tk.Label(self.root, text="Parsed Data:")
        self.data_label.pack(pady=10)

        self.data_text = tk.Text(self.root, height=5, width=40)
        self.data_text.pack(pady=10)

    def update_from_entry(self):
        data_string = self.entry.get()
        self.parse_data(data_string)

    def parse_data(self, data_string):
        if data_string:
            match = re.match(self.data_pattern, data_string)
            if match:
                self.data_text.delete(1.0, tk.END)
                self.data_text.insert(tk.END, f"A: {match.group(1)}, {match.group(2)}, {match.group(3)}")
                self.data_text.insert(tk.END, f"\nH: {match.group(4)}, {match.group(5)}, {match.group(6)}")
                self.data_text.insert(tk.END, f"\nB: {match.group(7)}, {match.group(8)}")
                self.data_text.insert(tk.END, f"\nQ: {match.group(9)}, {match.group(10)}, {match.group(11)}, {match.group(12)}")
                self.data_text.insert(tk.END, f"\nL: {match.group(13)}")
            else:
                self.data_text.delete(1.0, tk.END)
                self.data_text.insert(tk.END, "ParseErr")
        else:
            self.data_text.delete(1.0, tk.END)
            self.data_text.insert(tk.END, "No data entered")

    def update_data_from_serial(self, data_string):
        self.parse_data(data_string)

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorDashboard(root)

    try:
        ser = serial.Serial('COM3', 115200, timeout=0.02)
        print("Serial port opened successfully.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        ser = None

    def check_serial_data():
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').rstrip()
                    if line:
                        app.update_data_from_serial(line)
            except Exception as e:
                print(f"Error reading from serial: {e}")
        
        root.after(10, check_serial_data) 

    root.after(100, check_serial_data)

    root.mainloop()

    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
