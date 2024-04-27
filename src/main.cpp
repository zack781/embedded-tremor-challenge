#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_ZeroFFT.h>
#include "arduinoFFT.h"

#define NEOPIX_PIN    A2
#define NUM_PIXELS    5

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t samples = 64;
const double sampling = 40;

float vReal[samples];
float vImag[samples];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, sampling);

float N = 1; // Window size for moving average

void setup() {
  CircuitPlayground.begin();
  Serial.begin(115200);
  // Print log
  Serial.println("setup");

  // Configure the timer interrupt
  EICRA = 0b01000000; // Config for any edge
  
  // Configure 8-bit timer zero
  TCCR0A = 0b00000010; // CTC count to 0CR0A
  TCCR0B = 0b00000011; // Prescaler to 64 -> new tick is 8us

  TIMSK0 = 0b00000010; // Enable interrupt on compare match A
  OCR0A = 124; // 1ms clock cycle

  for (uint16_t i = 0; i < samples; i++) { // Default the arrays to 0s
    vReal[i] = 0;
    vImag[i] = 0;
  }
}

float calculateNpointMovingAVG(float* avg_vals) {
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += avg_vals[i];
  }
  return sum / N;  
}

ISR (TIMER0_COMPA_vect) {
  // This is ran every 1ms

  // This print format must be follow in order to display the data in Teloplot
  Serial.print(">X Value:");
  Serial.println(CircuitPlayground.motionX());
  Serial.print(">Y Value:");
  Serial.println(CircuitPlayground.motionY());
  Serial.print(">Z Value:");
  Serial.println(CircuitPlayground.motionZ());

  float xAccel = CircuitPlayground.motionX();
  float yAccel = CircuitPlayground.motionY();
  float zAccel = CircuitPlayground.motionZ();

  // Add the new data to the FFT
  for (uint16_t i = 0; i < samples - 1; i++) {
    vReal[i] = vReal[i + 1];
    vImag[i] = vImag[i + 1];
  }
  vReal[samples - 1] = xAccel;
  vImag[samples - 1] = 0;

  // Perform Moving Average
  float HRMovingAvg = calculateNpointMovingAVG(vReal);
  vReal[samples - 1] = HRMovingAvg;

  // Perform the FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
  FFT.compute(FFTDirection::Forward); // Compute FFT
  FFT.complexToMagnitude(); // Compute magnitudes
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
}