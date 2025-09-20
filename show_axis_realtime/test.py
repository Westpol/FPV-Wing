import serial
import matplotlib.pyplot as plt
from collections import deque

# ------------------------
# Configure your serial port
# in STMCUBEIDE add USB_PRINTLN("pitch:%f,roll:%f", imu_data.pitch_angle, imu_data.roll_angle); print function
# ------------------------
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# ------------------------
# Data storage
# ------------------------
max_samples = 200
pitch_vals = deque([0.0]*max_samples, maxlen=max_samples)
roll_vals  = deque([0.0]*max_samples, maxlen=max_samples)

# ------------------------
# Setup plot
# ------------------------
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot(range(max_samples), pitch_vals, label="Pitch")
line2, = ax.plot(range(max_samples), roll_vals, label="Roll")
ax.set_ylim(-180, 180)
ax.set_xlim(0, max_samples-1)
ax.set_title("FC Orientation (Pitch/Roll)")
ax.set_xlabel("Samples")
ax.set_ylabel("Degrees")
ax.legend()
fig.canvas.draw()
background = fig.canvas.copy_from_bbox(ax.bbox)

# ------------------------
# Main loop
# ------------------------
sample_count = 0
while True:
    line = ser.readline().decode(errors='ignore').strip()
    if line.startswith("pitch"):
        try:
            parts = line.split(",")
            pitch = float(parts[0].split(":")[1])
            roll  = float(parts[1].split(":")[1])

            # Append new values
            pitch_vals.append(pitch)
            roll_vals.append(roll)

            # Only redraw every few samples for performance
            sample_count += 1
            if sample_count % 2 == 0:  # redraw every 2 samples
                fig.canvas.restore_region(background)
                line1.set_ydata(pitch_vals)
                line2.set_ydata(roll_vals)
                ax.draw_artist(line1)
                ax.draw_artist(line2)
                fig.canvas.blit(ax.bbox)
                fig.canvas.flush_events()
        except Exception as e:
            # Catch any parsing errors silently
            pass

