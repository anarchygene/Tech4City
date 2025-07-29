#include "PlayAndRecord.h"
#include "AudioAnalyzer.h"
#include "Arduino.h"
#include "SPIFFS.h"
#include "driver/i2s.h"

void setupSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Failed to mount SPIFFS");
  } else {
    Serial.println("✅ SPIFFS mounted");
  }
}

void writeWAVHeader(File &file, int32_t dataSize) {
  const int32_t fileSize = dataSize + 36;
  const int32_t sampleRate = SAMPLE_RATE;
  const int32_t byteRate = SAMPLE_RATE * 2;
  const uint16_t blockAlign = 2;
  const uint16_t bitsPerSample = 16;
  const uint16_t audioFormat = 1;
  const uint16_t numChannels = 1;

  file.write((uint8_t*)"RIFF", 4);
  file.write((uint8_t*)&fileSize, 4);
  file.write((uint8_t*)"WAVE", 4);
  file.write((uint8_t*)"fmt ", 4);
  uint32_t subChunk1Size = 16;
  file.write((uint8_t*)&subChunk1Size, 4);
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&numChannels, 2);
  file.write((uint8_t*)&sampleRate, 4);
  file.write((uint8_t*)&byteRate, 4);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);
  file.write((uint8_t*)"data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

void recordAudio(AudioMode& audioMode) {
  File file = SPIFFS.open(RECORD_FILE, "w");
  if (!file) {
    Serial.println("❌ Could not create file");
    return;
  }

  Serial.println("🎙️ Recording...");

  // Write placeholder header
  uint32_t dummy[9];
  file.write((uint8_t*)dummy, 36);

  size_t bytes_read;
  int16_t data[BLOCK_SIZE];
  int32_t totalBytes = 0;

  unsigned long start = millis();
  while (millis() - start < MAX_RECORD_SECS * 1000) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("stop")) {
        Serial.println("🛑 Recording stopped by command");
        break;
      }
    }

    esp_err_t err = i2s_read(I2S_NUM_0, data, sizeof(data), &bytes_read, pdMS_TO_TICKS(10));
    if (err == ESP_OK && bytes_read > 0) {
      file.write((uint8_t*)data, bytes_read);
      totalBytes += bytes_read;
    }
  }

  // Finalize WAV header
  file.seek(0);
  writeWAVHeader(file, totalBytes);
  file.close();

  Serial.println("✅ Recording saved to SPIFFS");
  audioMode = IDLE;
}

void playAudio(AudioMode& audioMode) {
  File file = SPIFFS.open(RECORD_FILE, "r");
  if (!file) {
    Serial.println("❌ Could not open file for playback");
    audioMode = IDLE;
    return;
  }

  // Skip WAV header (44 bytes)
  file.seek(44);

  Serial.println("🔊 Playing...");

  uint8_t buffer[BLOCK_SIZE];
  while (file.available() && audioMode == PLAYING) {
    int bytesRead = file.read(buffer, BLOCK_SIZE);
    if (bytesRead > 0) {
      size_t bytesWritten;
      
      // Amplify each sample
      for (int i = 0; i < bytesRead / 2; i++) {
        int16_t sample = ((int16_t*)buffer)[i];
        int32_t amplified = sample * 2;  // Gain = 2x
        ((int16_t*)buffer)[i] = constrain(amplified, -32768, 32767);
      }

      i2s_write(I2S_NUM_1, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
    }
  }

  file.close();
  Serial.println("✅ Playback finished");
  audioMode = IDLE;
}