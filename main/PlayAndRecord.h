#ifndef PlayAndRecord_h
#define PlayAndRecord_h

#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
#include "driver/i2s.h"

// Constants
#define MAX_RECORD_SECS 10
#define RECORD_FILE "/recorded.wav"

// Enum for audio modes
enum AudioMode {
  IDLE,
  RECORDING,
  PLAYING
};

// Function declarations
void setupSPIFFS();
void writeWAVHeader(File &file, int32_t dataSize);
void recordAudio(AudioMode& audioMode);
void playAudio(AudioMode& audioMode);
void stopAlarm();

// External variables
extern bool waitingForReset;
extern bool guyFell;
extern unsigned long fallStartTime;

#endif