#ifndef AUDIO_ANALYZER_H
#define AUDIO_ANALYZER_H

// Define constants here so they are available to all files
#define FFT_SIZE 1024 
#define BLOCK_SIZE 512
#define SAMPLE_RATE 8000

// --- Publicly Accessible Variables ---

// Use 'extern' to declare that lastSpectrum is DEFINED in a .cpp file 
// but can be ACCESSED from other files (like your .ino).
extern double lastSpectrum[FFT_SIZE / 2];


// --- Public Functions ---
bool analyzeAudio();

#endif // AUDIO_ANALYZER_H
