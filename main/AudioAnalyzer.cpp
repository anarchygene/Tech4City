#include "Arduino.h"
#include "AudioAnalyzer.h"
#include "driver/i2s.h"
#include <arduinoFFT.h> // Correct include for the library


// --- Fall Detection Thresholds (requires tuning) ---
const double ENERGY_THRESHOLD = 150000.0; 
const double ZCR_THRESHOLD = 280.0;     
const double SPECTRAL_FLUX_THRESHOLD = 80000.0; 
const double LOW_FREQ_ENERGY_RATIO_THRESHOLD = 0.; // Changed name for clarity

// --- Buffers ---
int16_t i2sBuffer[FFT_SIZE];
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
double lastSpectrum[FFT_SIZE / 2];

// Instantiate the FFT object
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, FFT_SIZE, (double) SAMPLE_RATE);

// Feature Extraction Function
static void extract_features(double* spectrum, double& energy, double& zcr, double& spectralFlux, double& lowFreqEnergyRatio) {
    energy = 0;
    zcr = 0;
    spectralFlux = 0;
    double low_freq_sum = 0;

    // Calculate features from the first half of the spectrum
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        energy += spectrum[i];

        // Calculate spectral flux and update last spectrum
        double flux = spectrum[i] - lastSpectrum[i];
        if (flux > 0) {
            spectralFlux += flux;
        }
        lastSpectrum[i] = spectrum[i];
        
        // Assuming low frequencies are in the first 20% of the bins (e.g., up to 800Hz for 8kHz sample rate)
        if (i < (FFT_SIZE / 2) * 0.2) { 
            low_freq_sum += spectrum[i];
        }
    }
    
    // Calculate Zero-Crossing Rate from the original time-domain signal
    for (int i = 1; i < FFT_SIZE; i++) {
      if ((vReal[i-1] > 0 && vReal[i] <= 0) || (vReal[i-1] < 0 && vReal[i] >= 0)) {
        zcr++;
      }
    }

    if (energy > 0) {
        lowFreqEnergyRatio = low_freq_sum / energy;
    } else {
        lowFreqEnergyRatio = 0;
    }
}

bool analyzeAudio() {

  static String inputBuffer = "";

    size_t bytes_read;
    i2s_read(I2S_NUM_0, &i2sBuffer, sizeof(i2sBuffer), &bytes_read, portMAX_DELAY);
    
    if (bytes_read > 0) {
        // 1. Prepare data for FFT
        for (int i = 0; i < FFT_SIZE; i++) {
            vReal[i] = (double)i2sBuffer[i]; // Copy I2S data to double array
            vImag[i] = 0;                    // Ensure imaginary part is zero
        }

        // 2. Apply windowing
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

        // 3. Compute FFT
        FFT.compute(FFT_FORWARD);

        // 4. Calculate magnitude
        FFT.complexToMagnitude(); // Result is stored in vReal

        // 5. Extract features
        double energy, zcr, spectralFlux, lowFreqEnergyRatio;
        extract_features(vReal, energy, zcr, spectralFlux, lowFreqEnergyRatio);
        
        // --- Fall Detection Logic ---
        bool isHighEnergy = energy > ENERGY_THRESHOLD;
        bool isHighZCR = zcr > ZCR_THRESHOLD;
        bool isSuddenChange = spectralFlux > SPECTRAL_FLUX_THRESHOLD;
        bool isLowFreqDominant = lowFreqEnergyRatio > LOW_FREQ_ENERGY_RATIO_THRESHOLD;
        
        // For debugging:
        Serial.printf("Energy: %.2f, ZCR: %.2f, Flux: %.2f, LF Ratio: %.2f\n", energy, zcr, spectralFlux, lowFreqEnergyRatio);

        if (isHighEnergy && isSuddenChange && isLowFreqDominant) { // ZCR can be noisy, might remove it initially
            Serial.println("------------------------------------------------------------------------------------------------------------");
            Serial.println("🚨 POTENTIAL FALL DETECTED! 🚨");
            Serial.printf("Energy: %.2f, ZCR: %.2f, Flux: %.2f, LF Ratio: %.2f\n", energy, zcr, spectralFlux, lowFreqEnergyRatio);
            Serial.println("------------------------------------------------------------------------------------------------------------");
            return true;
        }
    }
    return false;
}
