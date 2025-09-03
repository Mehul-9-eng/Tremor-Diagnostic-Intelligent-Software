import serial
import serial.tools.list_ports
import pandas as pd
import time


def safe_float(s):
    try:
        return float(s)
    except:
        return None


# === CONFIGURATION ===
DEFAULT_PORT = "COM4"
BAUD = 115200
# =====================


print("Detecting serial ports...")
ports = list(serial.tools.list_ports.comports())
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} - {p.description}")


port_input = input(f"Port (press enter to use {DEFAULT_PORT}): ").strip()
if port_input == "":
    PORT = DEFAULT_PORT
elif port_input.isdigit():
    idx = int(port_input)
    if idx < 0 or idx >= len(ports):
        print("Invalid index; exiting.")
        exit(1)
    PORT = ports[idx].device
else:
    PORT = port_input


label = input("Enter label for this session (e.g. tremor or normal): ").strip()
duration = int(input("How many seconds to record? (e.g. 20): ").strip())
filename = f"{label}_{int(time.time())}.csv"


try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Failed to open port {PORT}: {e}")
    exit(1)


print(f"Connected to {PORT}. Recording for {duration}s with label '{label}'...")


data = []
start = time.time()
last_rms = None


while time.time() - start < duration:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line:
        continue
    parts = line.split(",")
    if parts[0] == "RMS" and len(parts) >= 4:
        rx = safe_float(parts[1])
        ry = safe_float(parts[2])
        rz = safe_float(parts[3])
        if rx is not None and ry is not None and rz is not None:
            last_rms = (rx, ry, rz)
    elif parts[0] == "FREQ" and len(parts) >= 4:
        fx = safe_float(parts[1])
        fy = safe_float(parts[2])
        fz = safe_float(parts[3])
        if fx is None or fy is None or fz is None:
            continue
        if last_rms is None:
            continue
        rx, ry, rz = last_rms
        data.append([rx, ry, rz, fx, fy, fz, label])


ser.close()
print(f"Saved {len(data)} samples to {filename}")
df = pd.DataFrame(data, columns=["rms_x", "rms_y", "rms_z", "freq_x", "freq_y", "freq_z", "label"])
df.to_csv(filename, index=False)
