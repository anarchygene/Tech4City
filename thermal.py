import serial
import numpy as np
import matplotlib.pyplot as plt
import time

# Configure serial port (change 'COMX' to your actual COM port)
ser = serial.Serial('COM4', 115200)
time.sleep(2)  # Wait for serial connection

plt.ion()
fig = plt.figure()

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            temps = list(map(float, line.split(',')))
            matrix = np.reshape(temps, (8, 8))

            plt.clf()
            plt.imshow(matrix, cmap='inferno', interpolation='bilinear')
            plt.colorbar(label='Temperature (°C)')
            plt.title("Thermal Camera (8x8)")
            plt.pause(0.1)

    except KeyboardInterrupt:
        break

plt.close()
ser.close()