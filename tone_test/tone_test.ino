#include <WiFi.h>
#include "Audio.h"

// Replace with your network credentials
const char* ssid = "Gene";
const char* password = "iemoosatie24881";

// I2S pins for MAX98357A Speaker
#define I2S_DOUT    8
#define I2S_BCLK    7
#define I2S_LRC     6

Audio audio;

void setup() {
  Serial.begin(921600);
  Serial.println("Connecting to WiFi...");
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Initialize I2S audio output
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); // Volume: 0-21 (adjust as needed)
  
  Serial.println("Connecting to MP3 stream...");
  
  // Try one of these test URLs:
  //audio.connecttoURL("http://stream.live.vcshouston.org:8000/live"); // Classical music
  audio.connecttohost("https://www.soundjay.com/misc/sounds/bell-ringing-05.mp3"); // Pop music
  //audio.connecttoURL("http://www.noiseaddicts.com/samples_1w72b820/2547.mp3"); // Short sample
}

void loop() {
  audio.loop();
  // Keep the WiFi connection alive
  delay(1);
}

// Optional callback functions for debugging
void audio_info(const char *info) {
  Serial.print("info: ");
  Serial.println(info);
}

void audio_id3data(const char *info) {
  Serial.print("id3: ");
  Serial.println(info);
}

void audio_eof_mp3(const char *info) {
  Serial.print("eof: ");
  Serial.println(info);
}