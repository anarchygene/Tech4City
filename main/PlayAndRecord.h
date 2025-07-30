#ifndef PlayAndRecord_h
#define PlayAndRecord_h

#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
#include "driver/i2s.h"

// Constants
#define MAX_RECORD_SECS 5
#define RECORD_FILE "/recorded.wav"
#define FALL_DETECTED_FILE "/fall_detected.wav"

// Enum for audio modes
enum AudioMode {
  IDLE,
  PLAYING
};

// Function declarations
void setupSPIFFS();
// void writeWAVHeader(File &file, int32_t dataSize);
// void recordAudio(AudioMode& audioMode);
// void playAudio(AudioMode& audioMode);
void playAudioTask(void *parameter);
void playFallDetected();

#endif