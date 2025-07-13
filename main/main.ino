#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "driver/i2s.h"
#include <math.h>

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
#define I2S_SPK_BCLK 7
#define I2S_SPK_LRC 6
#define I2S_SPK_DOUT 8


// ========================
// Constants & Settings
// ========================
#define SERIAL_BAUD 115200
#define SAMPLE_RATE 16000
#define BLOCK_SIZE 64
#define TONE_FREQ 440.0f  // A4 note
#define TONE_AMPLITUDE 3000

unsigned long thermalTimer = 0;
const long thermalInterval = 100;  // ~10 Hz

bool fall = false;
bool lying = false;
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

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Initialize I2C for AMG8833
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!amg.begin()) {
    Serial.println("Could not find AMG8833 sensor!");
    while (1)
      ;  // Halt if sensor not found
  }

  delay(100);  // Allow time for initialization

  // Initialize I2S for Microphone (RX)
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 8,
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
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
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
  int16_t buffer[256];
  for (int i = 0; i < 256; i++) {
    float t = (float)(i % SAMPLE_RATE) / SAMPLE_RATE;
    buffer[i] = TONE_AMPLITUDE * sinf(2.0f * PI * TONE_FREQ * t);
  }
  size_t bytes_written;
  i2s_write(I2S_NUM_1, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
}

// ========================
// Main Loop
// ========================
void loop() {
  unsigned long currentMillis = millis();

  // === THERMAL SENSOR ===
  if (false) {
    if (currentMillis - thermalTimer >= thermalInterval) {
      thermalTimer = currentMillis;

      amg.readPixels(pixels);

      // Format: T:p1,p2,...p64
      Serial.print("T:");
      for (int i = 0; i < 64; i++) {
        Serial.print(pixels[i]);
        if (i < 63) Serial.print(",");
      }
      Serial.println();


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

  // === MICROPHONE INPUT ===
  size_t bytes_read;
  int16_t micData[BLOCK_SIZE];

  esp_err_t err = i2s_read(I2S_NUM_0, micData, sizeof(micData), &bytes_read, pdMS_TO_TICKS(10));

  if (err == ESP_OK && bytes_read > 0) {
    int num_samples = bytes_read / sizeof(int16_t);

    for (int i = 0; i < num_samples; i++) {
      // Format: A:sample_value
      Serial.print("A:");
      Serial.println(micData[i]);
    }
  }

  // === SPEAKER OUTPUT ===
  generateAndPlayTone();  // You can comment this out or trigger via command if needed
}