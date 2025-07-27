import serial
import wave
import time

# === Configuration ===
PORT = 'COM4'           # Update to your COM port
BAUDRATE = 921600
WAV_FILE = 'recorded_audio.wav'  # Your recorded audio

# === Open Serial ===
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Allow time for serial connection
    print("✅ Connected to ESP32")
except Exception as e:
    print(f"❌ Could not open serial port: {e}")
    exit(1)

# === Send 'play' Command ===
print("🎮 Sending 'play' command to ESP32...")
ser.write(b'play\n')
time.sleep(0.5)  # Give ESP32 time to switch to PLAYING mode

# === Read and Send WAV File ===
try:
    with wave.open(WAV_FILE, 'rb') as wf:
        sample_rate = wf.getframerate()
        channels = wf.getnchannels()
        width = wf.getsampwidth()

        # Validate audio format
        if sample_rate != 16000:
            print("⚠️ Warning: Sample rate should be 16000 Hz")
        if channels != 1:
            print("❌ Only mono audio is supported")
            exit(1)
        if width != 2:
            print("❌ Only 16-bit audio is supported")
            exit(1)

        print("🔊 Streaming audio data to ESP32...")

        frame_count = 0
        while True:
            frames = wf.readframes(512)
            if not frames:
                break
            ser.write(frames)
            frame_count += len(frames)
            print(f"📤 Sent {frame_count} bytes...", end='\r')
            time.sleep(0.001)  # Small delay to avoid overwhelming serial

    print(f"\n✅ Audio file '{WAV_FILE}' sent successfully!")

except FileNotFoundError:
    print(f"❌ WAV file '{WAV_FILE}' not found!")
    exit(1)
except Exception as e:
    print(f"❌ Error reading or sending audio: {e}")
    exit(1)

# === Send 'stop' (Optional) ===
print("⏹ Sending 'stop' command...")
ser.write(b'stop\n')

# === Close Serial ===
ser.close()
print("👋 Serial connection closed.")