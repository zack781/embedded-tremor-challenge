#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define NEOPIX_PIN    A2
#define NUM_PIXELS    5
#define TIME 15 //the amount of time you would like to take for a test, in seconds

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t samples = 64;
const double sampling = 100; //the speed at which we take samples, 10ms

int tTime = TIME;
int testTime = (tTime/((1/sampling)*samples)); //calculates number of cycles necessary for (roughly) TIME seconds test time 

const uint16_t N = 6; // Window size for moving average
float vReal[samples] = {0}; // A buffer to hold the real values
float vImag[samples] = {0}; // A buffer to hold the imaginary values
float freqDataPoints[100]; //A buffer for frequency data points collected from the FFT
float MagnitudeDataPoints[100]; //A buffer for the magnitude of the frequencies collected from the FFT
float avgArray[N];

//declares FFT object
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, sampling);

//sums the entire power spectrum
float sumPower = 0;
//sums only the peak spectrum
float sumPeakPower = 0;

//checks the average after about 10 seconds(roughly 15 processes)
int processCounter = 0;

void setup() { 
  CircuitPlayground.begin();
  // Configure the timer interrupt
  EICRA = 0b01000000; // Config for any edge
  
  // Configure 8-bit timer zero
  TCCR0A = 0b00000010; // CTC count to 0CR0A
  TCCR0B = 0b00000101; // Prescaler to 1024 -> new tick is .128ms

  TIMSK0 = 0b00000010; // Enable interrupt on compare match A
  OCR0A = 78; // 10ms clock cycle
}

float calculateNpointMovingAVG(float avg_vals[]) {
  float sum = 0;
  for (int i = 0; i < N; i++) 
  {
    sum += avg_vals[i];
  }
  return sum / N;  
}

//finds approxmate area under the curve below two points using trapezoidal sum 
float findArea(float x1, float x2, float y1, float y2) 
{
  return (x2 - x1) * (y1 + y2) / 2;
}

void calcPower()
{
  //after adding points, calculate area under the entire domain (except 0-1 to remove gravity) and area under curve of interest
  for(int i = 0; i < 30; i++)
  {
    //within the range of interest
    if (freqDataPoints[i] >= 3 && freqDataPoints[i] <= 6.5) 
    {  
      float area = findArea(freqDataPoints[i], freqDataPoints[i+1], MagnitudeDataPoints[i], MagnitudeDataPoints[i+1]);
      sumPeakPower += area;
      sumPower += area;
    }
    // everywhere else where there isn't gravity (most prevalent from 0-2)
    else if(freqDataPoints[i] > 2)
    {
      float area = findArea(freqDataPoints[i], freqDataPoints[i+1], MagnitudeDataPoints[i], MagnitudeDataPoints[i+1]);
      sumPower += area;
    }
  }
}

//derived from the printVector function in the arduinoFFT examples
void dataProcess(float *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    abscissa = ((i * 1.0 * sampling) / samples);

    //adding points into vectors (32 of them)
    freqDataPoints[i] = abscissa;
    MagnitudeDataPoints[i] = vData[i];
  }
}

//used to fill the FFT array
int counter = 0;

//used to run moving average every 5 cycles of the ISR
int movingAvgCounter = 0;


ISR (TIMER0_COMPA_vect) {
  // This is ran every 5ms

  float xAccel = CircuitPlayground.motionX();
  float yAccel = CircuitPlayground.motionY();
  float zAccel = CircuitPlayground.motionZ();

  //magnitude of all three directions
  float accel = sqrt(xAccel * xAccel + yAccel * yAccel + zAccel * zAccel);

  // Add the new data to an array to calculate moving point average
  for (uint16_t i = 0; i < N - 1; i++) 
  {
    avgArray[i] = avgArray[i+1];
  }
  avgArray[N-1] = accel;
  movingAvgCounter++;
  //either adds moving average or real time value to the FFT array, moving average makes up about half the array
  if (movingAvgCounter == 1) 
  {
    movingAvgCounter=0;
    float HRMovingAvg = calculateNpointMovingAVG(avgArray);
    vReal[counter] = HRMovingAvg;
  } 
  
  else 
  {
    vReal[counter] = accel;
  }

  //no imaginary values, so 0 for all of them
  vImag[counter] = 0;
  
  //runs after vReal is filled
  if (counter == samples - 1) 
  {
    cli(); // Disable interrupts
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
    FFT.compute(FFTDirection::Forward); // Compute FFT
    FFT.complexToMagnitude(); // Compute magnitudes

    dataProcess(vReal, (samples >> 1));

    calcPower();

    counter = 0;

    //increments number of times area has been calculated
    processCounter++;
    sei(); // Enable interrupts
  }


  counter++;
}

void loop() {
  CircuitPlayground.clearPixels();

  //every ten cycles of finding total power(minus gravity) and power under the region of interest
  if(processCounter == testTime)
  {
    cli();
    sumPower = sumPower/processCounter;
    sumPeakPower = sumPeakPower/processCounter;

    //if the peak power is greater than 70% of the total power, then we consider it as a detection
    if((.7*(sumPower) <= (sumPeakPower)))
    {
      //Green Light
      CircuitPlayground.setPixelColor(0, 0x008000);
      CircuitPlayground.setPixelColor(1, 0x008000);
      CircuitPlayground.setPixelColor(2, 0x008000);
      CircuitPlayground.setPixelColor(3, 0x008000);
      CircuitPlayground.setPixelColor(4, 0x008000);
      CircuitPlayground.setPixelColor(5, 0x008000);
      CircuitPlayground.setPixelColor(6, 0x008000);
      CircuitPlayground.setPixelColor(7, 0x008000);
      CircuitPlayground.setPixelColor(8, 0x008000);
      CircuitPlayground.setPixelColor(9, 0x008000);
      delay(500000);
    }
    
    else
    {
      //red light
      CircuitPlayground.setPixelColor(0, 0xFF0000);
      CircuitPlayground.setPixelColor(1, 0xFF0000);
      CircuitPlayground.setPixelColor(2, 0xFF0000);
      CircuitPlayground.setPixelColor(3, 0xFF0000);
      CircuitPlayground.setPixelColor(4, 0xFF0000);
      CircuitPlayground.setPixelColor(5, 0xFF0000);
      CircuitPlayground.setPixelColor(6, 0xFF0000);
      CircuitPlayground.setPixelColor(7, 0xFF0000);
      CircuitPlayground.setPixelColor(8, 0xFF0000);
      CircuitPlayground.setPixelColor(9, 0xFF0000);
      delay(500000);
    }

    sumPower = 0;
    sumPeakPower = 0;
    processCounter = 0;
    CircuitPlayground.clearPixels();
    sei();
  }
  
}
