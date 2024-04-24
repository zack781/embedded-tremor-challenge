#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_ZeroFFT.h>



#define NEOPIX_PIN    A2
#define NUM_PIXELS    5

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  CircuitPlayground.begin();
  Serial.begin(115200);
  // Print log
  Serial.println("setup");
}

void loop() {
  CircuitPlayground.clearPixels();

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
  // Serial.println(CircuitPlayground.motionX());
  // Serial.println(CircuitPlayground.motionY());
  // Serial.println(CircuitPlayground.motionZ());

  // This print format must be follow in order to display the data in Teloplot
  Serial.print(">X Value:");
  Serial.println(CircuitPlayground.motionX());
  Serial.print(">Y Value:");
  Serial.println(CircuitPlayground.motionY());
  Serial.print(">Z Value:");
  Serial.println(CircuitPlayground.motionZ());

  delay(50);
}