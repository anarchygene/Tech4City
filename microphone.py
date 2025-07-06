import serial
import matplotlib.pyplot as plt
import numpy as np

# Update COM port accordingly
ser = serial.Serial('COM4', 115200)

plt.ion()
fig, ax = plt.subplots()
x = np.arange(0, 64)
line, = ax.plot(x, np.random.rand(64))
ax.set_ylim(-5000, 5000)

while True:
    data = []
    for _ in range(64):
        try:
            line_data = ser.readline().decode('utf-8').strip()
            if line_data:
                data.append(int(line_data))
        except:
            continue

    if len(data) == 64:
        line.set_ydata(data)
        fig.canvas.draw()
        fig.canvas.flush_events()