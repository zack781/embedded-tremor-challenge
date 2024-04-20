#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#define NEOPIX_PIN    A2
#define NUM_PIXELS    5

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  CircuitPlayground.begin();
}

void loop() {
  CircuitPlayground.clearPixels();
  delay(500);

  // Color can be set using RGB or Hex
  CircuitPlayground.setPixelColor(0, 255,   0,   0);
  CircuitPlayground.setPixelColor(1, 128, 128,   0);
  CircuitPlayground.setPixelColor(2,   0, 255,   0);
  CircuitPlayground.setPixelColor(3,   0, 128, 128);
  CircuitPlayground.setPixelColor(4,   0,   0, 255);
  
  CircuitPlayground.setPixelColor(5, 0xFF0000);
  CircuitPlayground.setPixelColor(6, 0x808000);
  CircuitPlayground.setPixelColor(7, 0x00FF00);
  CircuitPlayground.setPixelColor(8, 0x008080);
  CircuitPlayground.setPixelColor(9, 0x0000FF);

  // Reading Accelerometer Data
  Serial.println(CircuitPlayground.motionX());
  Serial.println(CircuitPlayground.motionY());
  Serial.println(CircuitPlayground.motionZ());

  delay(5000);
}