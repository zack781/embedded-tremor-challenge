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
float pk= 0;
float sumPower = 0;
float sumPeakPower = 0;
int processCounter = 0;
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


float freqDataPoints[100];
float MagnitudeDataPoints[100];
int dataPointsCounter = 0;

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 4; i < 9; i++)
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
    Serial.print(">Magnitude:");
    Serial.println(vData[i], 4);
    if (abscissa >= 3 && abscissa <= 6.5) {
      if (vData[i] * vData[i] > pk)
      {
        pk = vData[i] * vData[i];
      }
      freqDataPoints[dataPointsCounter] = abscissa;
      MagnitudeDataPoints[dataPointsCounter] = vData[i]*vData[i];
      dataPointsCounter++;
    }
  }
  Serial.println();
}

int counter = 0;
int movingAvgCounter = 0;

float findArea(float x1, float x2, float y1, float y2) {
  // Serial.print(">x1:");
  // Serial.println(x1, 4);
  // Serial.print(">x2:");
  // Serial.println(x2, 4);
  // Serial.print(">y1:");
  // Serial.println(y1, 4);
  // Serial.print(">y2:");
  // Serial.println(y2, 4);

  return (x2 - x1) * (y1 + y2) / 2;
}

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
  // float accel =  sqrt(xAccel * xAccel + yAccel * yAccel);

  // Add the new data to the FFT
  for (uint16_t i = 0; i < N - 1; i++) {
    avgArray[i] = avgArray[i+1];
  }
  avgArray[N-1] = accel;

  movingAvgCounter++;
  if (movingAvgCounter == 10) {
    movingAvgCounter=0;
    float HRMovingAvg = calculateNpointMovingAVG(avgArray);
    vReal[counter] = HRMovingAvg;
  } else {
    vReal[counter] = accel;
  }

  vImag[counter] = 0;
  
  if (counter == samples - 1) {
    cli(); // Disable interruptss
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
    FFT.compute(FFTDirection::Forward); // Compute FFT
    FFT.complexToMagnitude(); // Compute magnitudes

    Serial.println("Computed magnitudes:");
    PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

    float area = 0;
    for (int i = 0; i < dataPointsCounter - 1; i++) {
      // Serial.print(">freqDataPoints[i]:");
      // Serial.println(freqDataPoints[i], 4);
      area += findArea(freqDataPoints[i], freqDataPoints[i+1], MagnitudeDataPoints[i], MagnitudeDataPoints[i+1]);
    }

    Serial.print("Area under the curve: ");
    Serial.println(area);

    dataPointsCounter = 0;

    counter = 0;
    sei(); // Enable interrupts
  }

  counter++;
  processCounter++;
}

void loop() {
  if(processCounter == 1500)
  {
    processCounter = 0;
    if(sumPower/1500;
  }
  CircuitPlayground.clearPixels();

  // // Color can be set using RGB or Hex
  // CircuitPlayground.setPixelColor(0, 255,   0,   0);
  // CircuitPlayground.setPixelColor(1, 128, 128,   0);
  // CircuitPlayground.setPixelColor(2,   0, 255,   0);
  // CircuitPlayground.setPixelColor(3,   0, 128, 128);
  // CircuitPlayground.setPixelColor(4,   0,   0, 255);
  
  // CircuitPlayground.setPixelColor(5, 0xFF0000);
  // CircuitPlayground.setPixelColor(6, 0x808000);
  // CircuitPlayground.setPixelColor(7, 0x00FF00);
  // CircuitPlayground.setPixelColor(8, 0x008080);
  // CircuitPlayground.setPixelColor(9, 0x0000FF);
}