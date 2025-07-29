#ifndef AudioAnalyzer_h
#define AudioAnalyzer_h

#include "Arduino.h"
#include "driver/i2s.h"

// Define constants
#define SAMPLE_RATE 16000
#define ANALYSIS_BUFFER 512
#define BLOCK_SIZE 512

// Define the struct
struct AudioFeatures {
  float peakAmplitude;
  float rms;
  float loudness;
};

// Function declarations
AudioFeatures extractLoudSoundFeatures(int16_t* audioBuffer, int bufferIndex);
void detectFall(AudioFeatures features, unsigned long duration);
void processAudio(int16_t* audioBuffer, int& bufferIndex, bool& isRecording, unsigned long& recordStartTime);

// Constants
extern const int LOUDNESS_THRESHOLD;
extern const int PEAK_THRESHOLD;
extern const int MIN_SOUND_DURATION;
extern const int MAX_SOUND_DURATION;

#endif