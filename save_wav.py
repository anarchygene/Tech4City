import serial
import time

# === Configuration ===
PORT = 'COM4'  # Update to your COM port
BAUDRATE = 921600
OUTPUT_FILE = 'output.wav'

# === Start ===
print("Connecting to ESP32...")
ser = serial.Serial(PORT, BAUDRATE)
time.sleep(2)  # Wait for serial to initialize

print("Waiting for data...")
with open(OUTPUT_FILE, 'wb') as f:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            f.write(data)
            print(f"Received {f.tell()} bytes...", end='\r')
        # Stop after 10 seconds of silence or when connection ends
        if ser.in_waiting == 0 and f.tell() > 0:
            print("\nNo more data received. Saving file...")
            break

print(f"File saved as '{OUTPUT_FILE}'")