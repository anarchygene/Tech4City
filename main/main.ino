#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "driver/i2s.h"
#include "AudioAnalyzer.h"
#include "PlayAndRecord.h"
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

// External Reset Button
#define RESET_BUTTON_PIN 15
int resetButtonState = 0;

// ========================
// Constants & Settings
// ========================
#define SERIAL_BAUD 115200
#define MAX_RECORD_SECS 10
#define RECORD_FILE "/alarm.wav"

#define TONE_FREQ 440.0f  // A4 note
#define TONE_AMPLITUDE 3000

enum SystemState {
  LISTENING,
  SPEAKER
};
SystemState currentState = LISTENING;

// --- I2S Configuration Variables (Keep these global) ---
i2s_config_t i2s_mic_config;
i2s_pin_config_t i2s_mic_pins;

i2s_config_t i2s_spk_config;
i2s_pin_config_t i2s_spk_pins;

// ========================
// Global Objects
// ========================
Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];  // 8x8 = 64 pixels

// Thresholds (adjust based on testing)
const float HUMAN_TEMP_MIN = 30.5;    // Min temp for human detection [12]
const float ASPECT_RATIO_FALL = 1.1;  // If height/width < this, likely fallen [19]

// Variables for bounding box
int minX = 7, maxX = 0, minY = 7, maxY = 0;
bool previousFall = false;  // Track state changes

// Add these global variables
float prevAspectRatio = 0.0;
unsigned long prevTime = 0;
const float FALL_SPEED_THRESHOLD = 4;    // Change per second; tune based on testing
const unsigned long THERMAL_FRAME_INTERVAL = 100;  // ms between reads (adjust to sensor rate)

// Audio Modes
AudioMode audioMode = IDLE;

// Intervals variables
// ========================
unsigned long thermalTimer = 0;

unsigned long toneTimer = 0;
const long toneInterval = 1000;  // Play tone every 1 second
// ========================

// Silent timer
long silentWaitTime = 20000;      // 5 sec

// Fall detection states
bool fall = false;
bool lying = false;
long fallStartTime = 0;

unsigned long lyingDetectionCount = 0;
unsigned long standingDetectionCount = 0;

bool fallSoundDetected = false;  // Track if sound was detected
bool fallSoundPlayed = false;  // Track if sound was played
bool triggerAlarm = false;
bool alarmActive = false;

void detectThermalSensor() {
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
      
      lyingDetectionCount = 0;
      standingDetectionCount = 0;
    } 
    
    // Check if person currently standing
    if (isLying) {
      Serial.println("👤 Person lying. Aspect Ratio: " + String(aspectRatio));

      // Count lying frames detected
      lyingDetectionCount++;
      standingDetectionCount = 0;

    } else {
      Serial.println("✅ Person standing. Aspect Ratio: " + String(aspectRatio));
      
      // Count standing frames detected
      standingDetectionCount++;
      lyingDetectionCount = 0;
    } 

    // Check if person has been lying for 10 frames
    if (lyingDetectionCount >= 10) {
      lying = true;
    }

    // Check if person has been standing for 10 frames
    if (standingDetectionCount >= 10) {
      lying = false;
      fall = false;
      triggerAlarm = false;
      fallSoundDetected = false; // Reset sound detection when standing
    }

    // Update previous values
    previousFall = currentFall;
    prevAspectRatio = aspectRatio;
    prevTime = currentTime;
  } else {
    Serial.println("🐶 No human detected.");
    previousFall = false;
    lying = false;
    fall = false;
    triggerAlarm = false;
    fallSoundDetected = false;  // Reset sound detection when no human is detected
  }

  if (fall && lying) {
    Serial.println("🚩 Guy fell and lying down haha.");
    if (!triggerAlarm) fallStartTime = millis();
    triggerAlarm = true;
  }

  // delay(THERMAL_FRAME_INTERVAL);
}

// ========================
// Alarm Tone
// ========================
void playAlarm() {
  if (alarmActive) return;
  Serial.println("Playing Alarm");

  // i2s_start(I2S_NUM_1);

  audioMode = PLAYING;
  xTaskCreatePinnedToCore(
    playAudioTask,      // Task function
    "Audio Playback",   // Name
    8192,               // Stack size
    &audioMode,         // Parameter to pass
    1,                  // Priority
    NULL,               // Task handle (optional)
    1                   // Core ID (0 or 1)
  );

  alarmActive = true;
}

void stopAlarm() {
  if (!alarmActive) return;
  Serial.println("Stopping I2S");

  audioMode = IDLE;

  // i2s_stop(I2S_NUM_1);     // Stops the I2S driver
  // i2s_zero_dma_buffer(I2S_NUM_1);  // Clear any remaining data in DMA buffer

  alarmActive = false;
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

        if (cmd.equalsIgnoreCase("alarm")) {
          if (audioMode == IDLE) {
            audioMode = PLAYING;
            xTaskCreatePinnedToCore(
              playAudioTask,      // Task function
              "Audio Playback",   // Name
              8192,               // Stack size
              &audioMode,         // Parameter to pass
              1,                  // Priority
              NULL,               // Task handle (optional)
              1                   // Core ID (0 or 1)
            );
          } else {
            Serial.println("⚠️ Already recording or playing");
          }
        }
        else if (cmd.equalsIgnoreCase("play")) {
          if (audioMode == IDLE) {
            audioMode = PLAYING;
            playFallDetected();
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

          
          File file1 = SPIFFS.open(FALL_DETECTED_FILE, "r");
          if (file1) {
            Serial.printf("✅ File exists, size: %d bytes\n", file1.size());
            file1.close();
          } else {
            Serial.println("❌ No recording found");
          }
        }
        else if (cmd.equalsIgnoreCase("format")) {
          SPIFFS.format();
          Serial.println("🔧 SPIFFS formatted");
        }
        else if (cmd.equalsIgnoreCase("fell")) {
          Serial.println("The guy fell");
          triggerAlarm = true;
          fallStartTime = millis();
        }
        else if (cmd.equalsIgnoreCase("got up")) {
          fall = false;
          lying = false;
          triggerAlarm = false;
          Serial.println("The guy got up.");
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

void resetFallState() {
  fall = false;
  lying = false;
  triggerAlarm = false;
  fallSoundDetected = false;  // Reset sound detection
  fallSoundPlayed = false;  // Reset sound played state

  fallStartTime = 0;
  
  lyingDetectionCount = 0;
  standingDetectionCount = 0;
}

void switchToSpeakerMode() {
  if (currentState == SPEAKER) return;  // Already in speaker mode
  Serial.println("Switching to Speaker Mode...");
  
  // 1. Stop and uninstall the microphone driver
  i2s_driver_uninstall(I2S_NUM_0);

  // 2. Install and start the speaker driver
  i2s_driver_install(I2S_NUM_1, &i2s_spk_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &i2s_spk_pins);

  currentState = SPEAKER;
}

void switchToMicMode() {
  if (currentState == LISTENING) return;  // Already in mic mode
  Serial.println("Switching to Microphone Mode...");

  // 1. Stop the alarm and uninstall the speaker driver
  i2s_driver_uninstall(I2S_NUM_1);

  // 2. Install and start the microphone driver
  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  
  // 3. Re-initialize the spectrum buffer for clean analysis
  for (int i = 0; i < FFT_SIZE / 2; i++) {
    lastSpectrum[i] = 0;
  }

  currentState = LISTENING;
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
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C initialized");
  
  if (!amg.begin()) {
    Serial.println("Could not find AMG8833 sensor!");
    while (1);  // Halt if sensor not found
  }
  Serial.println("AMG8833 sensor found!");

  pinMode(RESET_BUTTON_PIN, INPUT);

  delay(100);  // Allow time for initialization

  // Initialize I2S for Microphone (RX)
  i2s_mic_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };

  i2s_mic_pins = {
    .bck_io_num = I2S_MIC_CLK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = -1,
    .data_in_num = I2S_MIC_SD
  };

  // Initialize I2S for Speaker (TX)
  i2s_spk_config = {
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

  i2s_spk_pins = {
    .bck_io_num = I2S_SPK_BCLK,
    .ws_io_num = I2S_SPK_LRC,
    .data_out_num = I2S_SPK_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // --- START IN LISTENING MODE ---
  Serial.println("System starting in LISTENING mode.");
  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  for (int i = 0; i < FFT_SIZE / 2; i++) {
      lastSpectrum[i] = 0;
  }

  currentState = LISTENING;
  Serial.println("System Initialized");
}

// ========================
// Main Loop
// ========================
void loop() {
  long currentMillis = millis();
  
  // === THERMAL SENSOR ===
  if (currentMillis - thermalTimer >= THERMAL_FRAME_INTERVAL) {
    detectThermalSensor();
    thermalTimer = currentMillis;
  }

  resetButtonState = digitalRead(RESET_BUTTON_PIN);

  // === Emergency silent timer ===
  if (triggerAlarm && fallSoundDetected) {
    switchToSpeakerMode();
    if (!fallSoundPlayed ) {
      playFallDetected();
      fallSoundPlayed = true;  // Prevent multiple plays
    }

    if (resetButtonState == HIGH) {
      resetFallState();
      Serial.println("✅ Reset button pressed - emergency cancelled.");
    } else if (currentMillis - fallStartTime >= silentWaitTime) {
      if (!alarmActive) {
        Serial.println("⏰ Silent timer expired → triggering alarm!");
        playAlarm();
      }
    } 

  } else {
    stopAlarm();
    switchToMicMode();
    fallSoundPlayed = false;  // Reset sound played state
  }

  handleCommands();
  // Process audio continuously
  if (currentState == LISTENING) {
    if (!fallSoundDetected) fallSoundDetected = analyzeAudio();
  }
}