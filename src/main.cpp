#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_ZeroFFT.h>
#include "arduinoFFT.h"

#define NEOPIX_PIN    A2
#define NUM_PIXELS    5

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t samples = 64;
const double sampling = 50;

float vReal[samples] = {0}; // A buffer to hold the real values
float vImag[samples] = {0}; // A buffer to hold the imaginary values

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, sampling);

const uint16_t N = 5; // Window size for moving average

float avgArray[N];

void setup() { 
  CircuitPlayground.begin();
  Serial.begin(115200);
  // Print log
  Serial.println("setup");

  // Configure the timer interrupt
  EICRA = 0b01000000; // Config for any edge
  
  // Configure 8-bit timer zero
  TCCR0A = 0b00000010; // CTC count to 0CR0A
  TCCR0B = 0b00000101; // Prescaler to 1024 -> new tick is 0.98ms

  TIMSK0 = 0b00000010; // Enable interrupt on compare match A
  OCR0A = 156; // 20ms clock cycle

  // for (uint16_t i = 0; i < samples; i++) { // Default the arrays to 0s
  //   vReal[i] = 0;
  //   vImag[i] = 0;
  // }
}

float calculateNpointMovingAVG(float avg_vals[]) {
  float sum = 0;
  for (int i = 0; i < N; i++) {
    // Serial.print("avg val = ");
    // Serial.println(avg_vals[i]);
    sum += avg_vals[i];
  }
  return sum / N;  
}

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / sampling);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * sampling) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    // Serial.print(">Magnitude:");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

int counter = 0;
int movingAvgCounter = 0;
ISR (TIMER0_COMPA_vect) {
  // This is ran every 20ms

  // This print format must be follow in order to display the data in Teloplot
  // Serial.print(">X Value:");
  // Serial.println(CircuitPlayground.motionX());
  // Serial.print(">Y Value:");
  // Serial.println(CircuitPlayground.motionY());
  // Serial.print(">Z Value:");
  // Serial.println(CircuitPlayground.motionZ());

  float xAccel = CircuitPlayground.motionX();
  float yAccel = CircuitPlayground.motionY();
  float zAccel = CircuitPlayground.motionZ();

  float accel = sqrt(xAccel * xAccel + yAccel * yAccel + zAccel * zAccel);

  // Add the new data to the FFT
  for (uint16_t i = 0; i < N - 1; i++) {
    avgArray[i] = avgArray[i+1];
  }
  avgArray[N-1] = accel;

  movingAvgCounter++;
  if (movingAvgCounter == 20) {
    movingAvgCounter=0;
    float HRMovingAvg = calculateNpointMovingAVG(avgArray);
    vReal[counter] = HRMovingAvg;
  } else {
    vReal[counter] = accel;
  }

  // avgArray[N-1] = xAccel;

  // for (uint16_t i = 0; i < samples; i++) {
  //   Serial.print("val = ");
  //   Serial.println(vReal[i]);
  // } 

  // Perform Moving Average
  // float HRMovingAvg = calculateNpointMovingAVG(avgArray);
  // Serial.print("HRMovingAvg = ");
  // Serial.println(HRMovingAvg);

  // for (uint16_t i = 0; i < samples - 1; i++) {
  //   Serial.print("vReal = ");
  //   Serial.println(vReal[i]);
  //   vReal[i] = vReal[i+1];
  // }
  // vReal[samples - 1] = HRMovingAvg;

  vImag[counter] = 0;
  counter++;

  if (counter == samples - 1) {
    cli(); // Disable interrupts
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
    FFT.compute(FFTDirection::Forward); // Compute FFT
    FFT.complexToMagnitude(); // Compute magnitudes

    // Serial.print(">FFT Value:");
    // Serial.println(FFT.majorPeak());

    Serial.println("Computed magnitudes:");
    PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

    counter = 0;
    sei(); // Enable interrupts
  }

  // for (uint16_t i = 0; i < samples; i++) {
  //   Serial.print("vReal = ");
  //   Serial.println(vReal[i]); 
  // }
 
  // Perform the FFT
  // FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
  // FFT.compute(FFTDirection::Forward); // Compute FFT
  // FFT.complexToMagnitude(); // Compute magnitudes

  // Serial.print(">FFT Value:");
  // Serial.println(FFT.majorPeak());
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