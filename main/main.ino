#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "driver/i2s.h"

// Dummy function
// const int ledPin = 8;

// void setup() {
//   pinMode(ledPin, OUTPUT);
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("ESP32-S3: Hello from setup()");
// }

// void loop() {
//   digitalWrite(ledPin, HIGH);
//   delay(500);
//   digitalWrite(ledPin, LOW);
//   delay(500);
//   Serial.println("Blink owo 212");
// }

// Thermal Image Function
// // Define custom I2C pins
// #define I2C_SDA 4
// #define I2C_SCL 5

// #define SERIAL_BAUD 115200

// Adafruit_AMG88xx amg;

// void setup() {
//   Serial.begin(SERIAL_BAUD);

//   // Initialize Wire with custom SDA and SCL pins
//   Wire.begin(I2C_SDA, I2C_SCL);

//   if (!amg.begin()) {
//     Serial.println("Could not find AMG8833 sensor!");
//     while (1); // Halt if sensor not found
//   }

//   delay(100); // Allow time for initialization
// }

// void loop() {
//   float pixels[64];
//   amg.readPixels(pixels);

//   // Output pixel temperatures as comma-separated values
//   for (int i = 0; i < 64; i++) {
//     Serial.print(pixels[i]);
//     if (i < 63) Serial.print(",");
//   }
//   Serial.println();

//   delay(1000); // ~10 FPS
// }


// Microphone Function
#define SAMPLE_RATE 16000
#define BLOCK_SIZE  64

// Define I2S Pins
#define I2S_WS      19
#define I2S_CLK     20
#define I2S_SD      21

void setup() {
  Serial.begin(115200);

  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 8,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_CLK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {
  size_t bytes_read;
  int16_t data[64];

  i2s_read(I2S_NUM_0, data, sizeof(data), &bytes_read, portMAX_DELAY);

  int num_samples = bytes_read / sizeof(int16_t);

  for (int i = 0; i < num_samples; i++) {
    Serial.println(data[i]);
  }
}