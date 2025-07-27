import serial
import matplotlib.pyplot as plt
import numpy as np

# Update COM port accordingly
ser = serial.Serial('COM4', 921600)

# Set up plot
plt.ion()
fig, ax = plt.subplots()
x = np.arange(0, 64)
line, = ax.plot(x, np.zeros(64), '-', lw=2)
ax.set_ylim(-32768, 32767)  # Signed 16-bit range
ax.set_title("Live Audio Waveform (INMP441)")
ax.set_xlabel("Sample")
ax.set_ylabel("Amplitude")

while True:
    data = []
    while len(data) < 64:
        try:
            line_data = ser.readline().decode('utf-8').strip()
            if line_data.startswith("A:"):
                value = int(line_data[2:])  # Remove "A:" and convert to int
                data.append(value)
        except Exception as e:
            print(f"Error: {e}")
            continue

    # Update plot
    line.set_ydata(data)
    fig.canvas.draw()
    fig.canvas.flush_events()