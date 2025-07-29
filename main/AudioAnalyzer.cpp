#include "AudioAnalyzer.h"
#include "Arduino.h"
#include "math.h"

// Define constants
const int LOUDNESS_THRESHOLD = 8000;
const int PEAK_THRESHOLD = 15000;
const int MIN_SOUND_DURATION = 50;
const int MAX_SOUND_DURATION = 500;

AudioFeatures extractLoudSoundFeatures(int16_t* audioBuffer, int bufferIndex) {
  AudioFeatures features = {0, 0, 0};
  
  if (bufferIndex < 10) return features; // Need enough data
  
  // 1. Peak Amplitude
  for (int i = 0; i < bufferIndex; i++) {
    if (abs(audioBuffer[i]) > features.peakAmplitude) {
      features.peakAmplitude = abs(audioBuffer[i]);
    }
  }
  
  // 2. RMS
  long sumSquares = 0;
  for (int i = 0; i < bufferIndex; i++) {
    sumSquares += (long)audioBuffer[i] * audioBuffer[i];
  }
  features.rms = sqrt((float)sumSquares / bufferIndex);
  
  // 3. Custom loudness metric (weighted combination)
  features.loudness = (features.rms * 0.7) + (features.peakAmplitude * 0.3);
  
  return features;
}

void detectFall(AudioFeatures features, unsigned long duration) {
  Serial.println("Analyzing sound event...");
  Serial.println("Duration: " + String(duration) + "ms");
  Serial.println("Peak: " + String(features.peakAmplitude));
  Serial.println("RMS: " + String(features.rms));
  Serial.println("Loudness: " + String(features.loudness));
  
  // Check if sound meets fall criteria
  bool isLoudEnough = (features.loudness > LOUDNESS_THRESHOLD) || (features.peakAmplitude > PEAK_THRESHOLD);
  bool isProperDuration = (duration >= MIN_SOUND_DURATION) && (duration <= MAX_SOUND_DURATION);
  
  if (isLoudEnough && isProperDuration) {
    Serial.println("🚨 FALL DETECTED! 🚨");
    Serial.println("=====================================");
    
    // Add your fall response code here:
    // - Send alert via WiFi/Bluetooth
    // - Trigger buzzer
    // - Log event
    // - Send notification
    
  } else {
    Serial.println("Sound detected but not classified as fall");
    if (!isLoudEnough) Serial.println("- Not loud enough");
    if (!isProperDuration) Serial.println("- Duration outside range");
    Serial.println("=====================================");
  }
}

void processAudio(int16_t* audioBuffer, int& bufferIndex, bool& isRecording, unsigned long& recordStartTime) {
  size_t bytes_read;
  int16_t micData[BLOCK_SIZE];
  
  esp_err_t result = i2s_read(I2S_NUM_0, &micData, sizeof(micData), &bytes_read, pdMS_TO_TICKS(10));
  
  if (result == ESP_OK && bytes_read > 0) {
    int num_samples = bytes_read / sizeof(int16_t);
    
    // Calculate energy for voice activity detection
    long energy = 0;
    for (int i = 0; i < num_samples; i++) {
      energy += abs(micData[i]);
    }
    float avg_energy = energy / num_samples;
    
    // Lower threshold for general sound detection
    const int SOUND_THRESHOLD = 50;
    
    if (avg_energy > SOUND_THRESHOLD) {
      // Start or continue recording
      if (!isRecording) {
        isRecording = true;
        recordStartTime = millis();
        bufferIndex = 0;
        Serial.println("🔊 Sound detected, analyzing...");
      }
      
      // Add samples to buffer
      for (int i = 0; i < num_samples && bufferIndex < ANALYSIS_BUFFER; i++) {
        audioBuffer[bufferIndex++] = micData[i];
      }
      
      // Force analysis if buffer is full
      if (bufferIndex >= ANALYSIS_BUFFER) {
        unsigned long soundDuration = millis() - recordStartTime;
        AudioFeatures features = extractLoudSoundFeatures(audioBuffer, bufferIndex);
        detectFall(features, soundDuration);
        isRecording = false;
        bufferIndex = 0;
      }
      
    } else if (isRecording) {
      // Sound stopped - analyze what we recorded
      unsigned long soundDuration = millis() - recordStartTime;
      
      if (soundDuration > 20) { // Minimum viable sound
        AudioFeatures features = extractLoudSoundFeatures(audioBuffer, bufferIndex);
        detectFall(features, soundDuration);
      }
      
      isRecording = false;
      bufferIndex = 0;
    }
  }

  delay(100);
}