import serial
import numpy as np
import matplotlib.pyplot as plt
import time

# Configure serial port (change 'COMX' to your actual COM port)
ser = serial.Serial('COM4', 921600)
time.sleep(2)  # Wait for serial connection

plt.ion()
fig = plt.figure()

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        
        # Only process lines that start with "T:"
        if line.startswith("T:"):
            # Remove the "T:" prefix and split the rest
            data_part = line[2:]  # everything after "T:"
            temps = list(map(float, data_part.split(',')))


            # Reshape into 8x8 grid
            matrix = np.reshape(temps, (8, 8))

            # Plot
            plt.clf()
            plt.imshow(matrix, cmap='inferno', interpolation='bilinear')
            plt.colorbar(label='Temperature (°C)')
            plt.title("Thermal Camera (8x8)")
            plt.pause(0.05)  # Update faster

    except KeyboardInterrupt:
        break

plt.close()
ser.close()