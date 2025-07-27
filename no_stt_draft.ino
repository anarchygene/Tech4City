#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "driver/i2s.h"
// #include <WiFi.h>         // Uncomment when enabling WiFi
// #include <HTTPClient.h>  // Uncomment for Huawei SMS alerts

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

// External Reset Button
#define RESET_BUTTON_PIN 15

// ========================
// Constants & Settings
// ========================
#define SERIAL_BAUD 921600
#define SAMPLE_RATE 16000
#define BLOCK_SIZE 512
#define MAX_RECORD_SECS 10
#define RECORD_FILE "/recorded.wav"

// Existing tone settings
#define TONE_FREQ 440.0f  
#define TONE_AMPLITUDE 3000

// Fall detection thresholds
const float HUMAN_TEMP_MIN = 25.0;
const float ASPECT_RATIO_FALL = 1.1;
const float FALL_SPEED_THRESHOLD = 4;
const unsigned long FRAME_INTERVAL = 100;

// Silent timer + escalation
unsigned long silentWaitTime = 120000;      // 2 min
unsigned long alarmEscalateTime = 300000;   // 5 min

// ========================
// Global State Variables
// ========================
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE]; // 8x8 grid

// Bounding box & fall detection
int minX = 7, maxX = 0, minY = 7, maxY = 0;
bool previousFall = false;
float prevAspectRatio = 0.0;
unsigned long prevTime = 0;

// Fall detection states
bool fallDetected = false;
bool waitingForReset = false;
bool alarmActive = false;
bool lying = false;
unsigned long fallStartTime = 0;
unsigned long alarmStartTime = 0;

// ========================
// SPIFFS + Audio Recording Vars
// ========================
bool isSpeaking = false;
int detectCount = 0;
int quietCount = 0;

// Audio Modes
enum AudioMode { IDLE, RECORDING, PLAYING };
AudioMode audioMode = IDLE;

unsigned long thermalTimer = 0;
const long thermalInterval = 100;  // ~10 Hz
unsigned long toneTimer = 0;
const long toneInterval = 1000;    // Play tone every 1s

// ========================
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
// WAV Header Writer
// ========================
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

// ========================
// Fall Detection Logic
// ========================
void detectFallFromThermal() {
  amg.readPixels(pixels);
  minX = 7; maxX = 0; minY = 7; maxY = 0;
  bool humanDetected = false;

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
    float timeDelta = (currentTime - prevTime) / 1000.0;
    float ratioChangeSpeed = (aspectRatio - prevAspectRatio) / timeDelta;

    bool isLying = aspectRatio < ASPECT_RATIO_FALL;
    bool currentFall = isLying && (abs(ratioChangeSpeed) > FALL_SPEED_THRESHOLD);

    if (currentFall && !previousFall) {
      Serial.println("⚠️ FALL DETECTED!");
      fallDetected = true;
      waitingForReset = true;
      fallStartTime = millis();
    }

    previousFall = currentFall;
    prevAspectRatio = aspectRatio;
    prevTime = currentTime;
  } else {
    previousFall = false;
  }
}

// ========================
// Reset Button Logic
// ========================
bool resetButtonPressed() {
  return digitalRead(RESET_BUTTON_PIN) == LOW;
}

// ========================
// Alarm Tone
// ========================
void playAlarmTone() {
  int16_t buffer[256];
  for (int i = 0; i < 256; i++) {
    float t = (float)(i % SAMPLE_RATE) / SAMPLE_RATE;
    buffer[i] = TONE_AMPLITUDE * sinf(2.0f * PI * TONE_FREQ * t);
  }
  size_t bytes_written;
  i2s_write(I2S_NUM_1, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
}

// ========================
// Record Audio
// ========================
void recordAudio() {
  File file = SPIFFS.open(RECORD_FILE, "w");
  if (!file) {
    Serial.println("❌ Could not create file");
    return;
  }

  Serial.println("🎙️ Recording...");
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
      if (cmd.equalsIgnoreCase("stop")) break;
    }
    esp_err_t err = i2s_read(I2S_NUM_0, data, sizeof(data), &bytes_read, pdMS_TO_TICKS(10));
    if (err == ESP_OK && bytes_read > 0) {
      file.write((uint8_t*)data, bytes_read);
      totalBytes += bytes_read;
    }
  }

  file.seek(0);
  writeWAVHeader(file, totalBytes);
  file.close();
  Serial.println("✅ Recording saved");
  audioMode = IDLE;
}

// ========================
// Playback
// ========================
void playAudio() {
  File file = SPIFFS.open(RECORD_FILE, "r");
  if (!file) {
    Serial.println("❌ Could not open file for playback");
    audioMode = IDLE;
    return;
  }

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
// Serial Commands
// ========================
void handleCommands() {
  static String inputBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        String cmd = inputBuffer;
        cmd.trim();  
        inputBuffer = "";
        if (cmd.equalsIgnoreCase("record")) {
          if (audioMode == IDLE) { audioMode = RECORDING; recordAudio(); }
        }
        else if (cmd.equalsIgnoreCase("play")) {
          if (audioMode == IDLE) { audioMode = PLAYING; playAudio(); }
        }
        else if (cmd.equalsIgnoreCase("stop")) {
          if (audioMode != IDLE) audioMode = IDLE;
        }
        else if (cmd.equalsIgnoreCase("info")) {
          File file = SPIFFS.open(RECORD_FILE, "r");
          if (file) { Serial.printf("✅ File exists, size: %d bytes\n", file.size()); file.close(); }
          else Serial.println("❌ No recording found");
        }
        else if (cmd.equalsIgnoreCase("format")) {
          SPIFFS.format();
          Serial.println("🔧 SPIFFS formatted");
        }
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 64) inputBuffer = "";
    }
  }
}

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Starting setup...");

  setupSPIFFS();
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!amg.begin()) {
    Serial.println("Could not find AMG8833 sensor!");
    while (1);
  }

  // Mic Config
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

  // Speaker Config
  i2s_config_t spk_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false
  };
  i2s_pin_config_t spk_pins = {
    .bck_io_num = I2S_SPK_BCLK,
    .ws_io_num = I2S_SPK_LRC,
    .data_out_num = I2S_SPK_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_1, &spk_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &spk_pins);

  Serial.println("System Initialized");
}

// ========================
// Main Loop
// ========================
void loop() {
  unsigned long now = millis();

  // Handle existing record/play commands
  handleCommands();

  // === Fall detection every frame ===
  if (now - thermalTimer >= thermalInterval) {
    thermalTimer = now;
    detectFallFromThermal();
  }

  // === Emergency silent timer ===
  if (waitingForReset && !alarmActive) {
    if (resetButtonPressed()) {
      waitingForReset = false;
      fallDetected = false;
      Serial.println("✅ Reset button pressed - emergency cancelled.");
    } else if (now - fallStartTime > silentWaitTime) {
      alarmActive = true;
      alarmStartTime = millis();
      Serial.println("⏰ Silent timer expired → triggering alarm!");
      // sendHuaweiAlert("⚠️ Elderly fell and is unresponsive!");
    }
  }

  // === Alarm escalation ===
  if (alarmActive) {
    playAlarmTone();
    if (resetButtonPressed()) {
      alarmActive = false;
      waitingForReset = false;
      fallDetected = false;
      Serial.println("✅ Alarm stopped by reset button.");
    } else if (now - alarmStartTime > alarmEscalateTime) {
      Serial.println("🔁 Alarm escalation: sending repeated alert...");
      // sendHuaweiAlert("⚠️ Elderly still unresponsive, immediate help required!");
      alarmStartTime = millis(); // restart escalation timer
    }
  }

  // Existing tone timer logic
  if (millis() - toneTimer >= toneInterval) {
    toneTimer = millis();
    // (you can still trigger regular tones here)
  }
}
