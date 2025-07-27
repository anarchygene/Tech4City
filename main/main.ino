#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "driver/i2s.h"
// #include "ekzl-project-1_inferencing.h"
// #include <math.h>
// #include <vector>

// ========================
// Pin Definitions
// ========================

// I2C for AMG8833 Thermal Sensor
#define I2C_SDA 4
#define I2C_SCL 5

// I2S for INMP441 Microphone (Input)
#define I2S_MIC_WS 13
#define I2S_MIC_CLK 14
#define I2S_MIC_SD 12

// I2S for MAX98357A Speaker (Output)
#define I2S_SPK_LRC 6
#define I2S_SPK_BCLK 7
#define I2S_SPK_DOUT 8

// ========================
// Constants & Settings
// ========================
#define SERIAL_BAUD 921600
#define SAMPLE_RATE 16000
#define BLOCK_SIZE 512
#define MAX_RECORD_SECS 10
#define RECORD_FILE "/recorded.wav"

#define TONE_FREQ 440.0f  // A4 note
#define TONE_AMPLITUDE 3000

// ========================
// Global Objects
// ========================
Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];  // 8x8 = 64 pixels

// Thresholds (adjust based on testing)
const float HUMAN_TEMP_MIN = 25.0;    // Min temp for human detection [12]
const float ASPECT_RATIO_FALL = 1.1;  // If height/width < this, likely fallen [19]

// Variables for bounding box
int minX = 7, maxX = 0, minY = 7, maxY = 0;
bool previousFall = false;  // Track state changes

// Add these global variables
float prevAspectRatio = 0.0;
unsigned long prevTime = 0;
const float FALL_SPEED_THRESHOLD = 4;    // Change per second; tune based on testing
const unsigned long FRAME_INTERVAL = 100;  // ms between reads (adjust to sensor rate)

bool isSpeaking = false;
int detectCount = 0;
int quietCount = 0;

// Audio Modes
enum AudioMode { IDLE, RECORDING, PLAYING };
AudioMode audioMode = IDLE;

#define THRESHOLD 100
#define BUFFER_SIZE 128  // Small buffer for pattern matching

// Global variables
// ========================
unsigned long thermalTimer = 0;
const long thermalInterval = 100;  // ~10 Hz

unsigned long toneTimer = 0;
const long toneInterval = 1000;  // Play tone every 1 second

bool fall = false;
bool lying = false;
// Setup SPIFFS
// ========================
void setupSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Failed to mount SPIFFS");
  } else {
    Serial.println("✅ SPIFFS mounted");
  }
}

// ========================
// Write WAV Header
// ========================
void writeWAVHeader(File &file, int32_t dataSize) {
  const int32_t fileSize = dataSize + 36;
  const int32_t sampleRate = SAMPLE_RATE;  // ← Make it a variable
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
  file.write((uint8_t*)&sampleRate, 4);     // ✅ Now &sampleRate is valid
  file.write((uint8_t*)&byteRate, 4);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);
  file.write((uint8_t*)"data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

// ========================
// Record Audio to SPIFFS
// ========================
void recordAudio() {
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
      String cmd = Serial.readStringUntil('\n');  // Read first
      cmd.trim();  // Trim in-place
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

// ========================
// Play Audio from SPIFFS
// ========================
void playAudio() {
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
      i2s_write(I2S_NUM_1, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
    }
  }

  file.close();
  Serial.println("✅ Playback finished");
  audioMode = IDLE;
}

// ========================
// Handle Serial Commands
// ========================
void handleCommands() {
  static String inputBuffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        String cmd = inputBuffer;  // Copy first
        cmd.trim();  
        inputBuffer = "";

        if (cmd.equalsIgnoreCase("record")) {
          if (audioMode == IDLE) {
            audioMode = RECORDING;
            recordAudio();
          } else {
            Serial.println("⚠️ Already recording or playing");
          }
        }
        else if (cmd.equalsIgnoreCase("play")) {
          if (audioMode == IDLE) {
            audioMode = PLAYING;
            playAudio();
          } else {
            Serial.println("⚠️ Already recording or playing");
          }
        }
        else if (cmd.equalsIgnoreCase("stop")) {
          if (audioMode != IDLE) {
            audioMode = IDLE;
            Serial.println("🛑 Stopped");
          }
        }
        else if (cmd.equalsIgnoreCase("info")) {
          File file = SPIFFS.open(RECORD_FILE, "r");
          if (file) {
            Serial.printf("✅ File exists, size: %d bytes\n", file.size());
            file.close();
          } else {
            Serial.println("❌ No recording found");
          }
        }
        else if (cmd.equalsIgnoreCase("format")) {
          SPIFFS.format();
          Serial.println("🔧 SPIFFS formatted");
        }
        else {
          Serial.println("Available commands: record, play, stop, info, format");
        }
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 64) inputBuffer = ""; // Prevent overflow
    }
  }
}

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Starting setup...");

  // Initialize SPIFFS
  setupSPIFFS();

  // Initialize I2C for AMG8833
  // Wire.begin(I2C_SDA, I2C_SCL);
  // Serial.println("I2C initialized");
  
  // if (!amg.begin()) {
  //   Serial.println("Could not find AMG8833 sensor!");
  //   while (1);  // Halt if sensor not found
  // }
  // Serial.println("AMG8833 sensor found!");

  // delay(100);  // Allow time for initialization

  // Initialize I2S for Microphone (RX)
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 16,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false
  };

  i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_CLK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);

  // Initialize I2S for Speaker (TX)
  i2s_config_t i2s_spk_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    // .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false
  };

  i2s_pin_config_t i2s_spk_pins = {
    .bck_io_num = I2S_SPK_BCLK,
    .ws_io_num = I2S_SPK_LRC,
    .data_out_num = I2S_SPK_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_1, &i2s_spk_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &i2s_spk_pins);

  Serial.println("System Initialized");
}

// ========================
// Tone Generation
// ========================
void generateAndPlayTone() {
  Serial.println("Playing tone");
  int16_t buffer[256];
  for (int i = 0; i < 256; i++) {
    float t = (float)(i % SAMPLE_RATE) / SAMPLE_RATE;
    buffer[i] = TONE_AMPLITUDE * sinf(2.0f * PI * TONE_FREQ * t);
  }
  size_t bytes_written;
  i2s_write(I2S_NUM_1, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
}
void playBeep() {
  const int sampleRate = 16000;
  const int frequency = 800;  // Higher pitch beep
  const int durationMs = 200; // 200ms beep
  const int samples = (sampleRate * durationMs) / 1000;
  int16_t buffer[256];
  
  for (int i = 0; i < samples; i += 256) {
    int chunkSize = (samples - i) > 256 ? 256 : (samples - i);
    for (int j = 0; j < chunkSize; j++) {
      float t = (float)(i + j) / sampleRate;
      buffer[j] = 1000 * sinf(2.0f * PI * frequency * t);
    }
    size_t bytes_written;
    i2s_write(I2S_NUM_1, buffer, chunkSize * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }
  
  // Play silence for 300ms
  memset(buffer, 0, sizeof(buffer));
  for (int i = 0; i < (sampleRate * 300) / 1000; i += 256) {
    int chunkSize = ((sampleRate * 300) / 1000 - i) > 256 ? 256 : ((sampleRate * 300) / 1000 - i);
    size_t bytes_written;
    i2s_write(I2S_NUM_1, buffer, chunkSize * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }
}
void playTone() {
  int16_t buffer[256];
  for (int i = 0; i < 256; i++) {
    buffer[i] = 3000 * sinf(2.0f * PI * 880.0 * i / 16000);
  }
  for (int n = 0; n < 50; n++) {
    size_t bytesWritten;
    i2s_write(I2S_NUM_1, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
  }
}

// ========================
// Main Loop
// ========================
void loop() {
  if (false) {
  unsigned long currentMillis = millis();

  // === THERMAL SENSOR ===
  if (currentMillis - thermalTimer >= thermalInterval) {
    thermalTimer = currentMillis;

    amg.readPixels(pixels);

    // Format: T:p1,p2,...p64
    // Serial.print("T:");
    // for (int i = 0; i < 64; i++) {
    //   Serial.print(pixels[i]);
    //   if (i < 63) Serial.print(",");
    // }
    // Serial.println();


    // Reset bounding box
    minX = 7;
    maxX = 0;
    minY = 7;
    maxY = 0;
    bool humanDetected = false;

    // Scan 8x8 grid for hot pixels
    for (int y = 0; y < 8; y++) {
      for (int x = 0; x < 8; x++) {
        int index = y * 8 + x;
        if (pixels[index] > HUMAN_TEMP_MIN) {
          humanDetected = true;
          if (x < minX) minX = x;
          if (x > maxX) maxX = x;
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
      }
    }

    if (humanDetected) {
      int width = maxX - minX + 1;
      int height = maxY - minY + 1;
      float aspectRatio = (float)height / (float)width;
      unsigned long currentTime = millis();

      float timeDelta = (currentTime - prevTime) / 1000.0;  // Seconds
      float ratioChangeSpeed = (aspectRatio - prevAspectRatio) / timeDelta;

      bool isLying = aspectRatio < ASPECT_RATIO_FALL;
      bool currentFall = isLying && (abs(ratioChangeSpeed) > FALL_SPEED_THRESHOLD);

      // Check for state change (fall event)
      if (currentFall && !previousFall) {
        Serial.println("⚠️ FALL DETECTED! Aspect Ratio: " + String(aspectRatio) + ", Speed of change: " + String(ratioChangeSpeed));
        fall = true;
        // Add alert code here, e.g., buzzer or LED
      } 
      
      if (isLying) {
        Serial.println("👤 Person lying. Aspect Ratio: " + String(aspectRatio));
        lying = true;
      } else {
        Serial.println("✅ Person standing. Aspect Ratio: " + String(aspectRatio));
        lying = false;
        fall = false;
      } 

      // Update previous values
      previousFall = currentFall;
      prevAspectRatio = aspectRatio;
      prevTime = currentTime;
    } else {
      Serial.println("❌ No human detected.");
      previousFall = false;
      lying = false;
      fall = false;
    }

    if (fall && lying) {
      Serial.println("🎨 Fell and lying down haha.");
    } else {
      Serial.println("Something else");
    }

    delay(FRAME_INTERVAL);
    
  }
  }

  // === SPEAKER OUTPUT ===
  if (millis() - toneTimer >= toneInterval) {
    toneTimer = millis();
    // generateAndPlayTone();
    // playBeep();
    // playTone();
  }
  handleCommands();
}