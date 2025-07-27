import serial
import numpy as np
import wave
import time
from scipy.signal import butter, sosfilt

def normalize_audio(data):
    """Normalize audio to max amplitude (prevents clipping)"""
    abs_max = np.abs(data).max()
    if abs_max == 0:
        return data
    normalized = data / abs_max
    return np.int16(normalized * 32767)

def butter_lowpass(cutoff, fs, order=5):
    """Design a low-pass filter"""
    sos = butter(order, cutoff, fs=fs, btype='low', output='sos')
    return sos

def lowpass_filter(data, cutoff=4000, fs=16000):
    """Apply low-pass filter to audio"""
    sos = butter_lowpass(cutoff, fs)
    filtered = sosfilt(sos, data)
    return np.int16(filtered)

def apply_gain(data, gain=8.0):
    """Boost volume (careful not to clip)"""
    boosted = data * gain
    return np.int16(np.clip(boosted, -32767, 32767))

# === Configuration ===
PORT = 'COM4'            # Update to your COM port
BAUDRATE = 921600        # Must match Arduino
SAMPLE_RATE = 16000      # Must match Arduino
SAMPLE_WIDTH = 2         # 16-bit PCM
CHANNELS = 1             # Mono
DURATION = 10            # Seconds to record

# File to save
WAV_FILE = "recorded_audio.wav"
MP3_FILE = "recorded_audio.mp3"

def record_audio(ser, duration_seconds=10):
    print("Sending record command...")
    ser.write(b'record\n')

    print(f"Recording for {duration_seconds} seconds...")
    raw_data = b''

    start_time = time.time()
    while time.time() - start_time < duration_seconds:
        # data = ser.read(ser.in_waiting or 1)  # Read available data
        data = ser.read(4096)  # Read 4KB at a time
        if data:
            raw_data += data
        time.sleep(0.001)  # Small delay to avoid CPU overuse

    print("Recording finished.")
    return raw_data

def save_wav_file(data, filename, sample_rate, channels, sample_width):
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)
        wf.writeframes(data)
    print(f"WAV file saved as '{filename}'")

if __name__ == "__main__":
    # Open serial connection
    ser = serial.Serial(PORT, BAUDRATE)
    time.sleep(2)  # Wait for serial to initialize

    # Record audio
    audio_data = record_audio(ser, DURATION)

    # Convert to NumPy array
    audio_array = np.frombuffer(audio_data, dtype=np.int16)

    # Apply processing in correct order
    print("Applying audio post-processing...")
    audio_filtered = lowpass_filter(audio_array)
    audio_boosted = apply_gain(audio_filtered, gain=8.0)   # High gain
    audio_final = normalize_audio(audio_boosted)           # Maximize volume

    # Save to WAV
    save_wav_file(audio_final, WAV_FILE, SAMPLE_RATE, CHANNELS, SAMPLE_WIDTH)

    # Close serial
    ser.close()

    print("✅ Done! You can now play the WAV file.")