#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define NEOPIX_PIN    A2
#define NUM_PIXELS    5

const uint8_t spBAD[]       PROGMEM = {0x08,0x00,0x71,0xC2,0x98,0xD1,0xA7,0xA9,0x5A,0xAA,0x13,0x14,0x15,0x1C,0x69,0xAD,0x4F,0x30,0xEC,0x88,0xAB,0x2F,0x3A,0xC1,0xB0,0x23,0x6E,0x51,0xEB,0x04,0xC3,0x9E,0x84,0x44,0xA3,0x13,0xF5,0xD0,0x16,0x16,0xB1,0x4E,0xDC,0x63,0x7B,0xA8,0xC7,0x3A,0x59,0x8F,0xED,0xA9,0x51,0xEB,0x54,0xAD,0x66,0x84,0x45,0xE5,0x55,0xB7,0x9C,0x91,0xE6,0xB5,0x57,0xDB,0x62,0x78,0x99,0xD7,0x6E,0x7D,0xCD,0xE6,0xEE,0x59,0xA5,0xF5,0x25,0xBA,0x85,0x57,0x99,0xD2,0x97,0xA0,0x11,0xD6,0x75,0xC2,0x98,0xBC,0x66,0xD6,0x46,0xD6,0x23,0x19,0x85,0x72,0x77,0x11,0xD9,0xE0,0xFA,0x6D,0xDC,0xC5,0xD4,0x4D,0xEA,0x9B,0x17,0x77,0x73,0xDB,0xFF,0x39,0x51,0x51,0x75,0x15,0x51,0x21};
const uint8_t spGOOD[]      PROGMEM = {0xA9,0x2C,0xA9,0xC5,0xD4,0x6C,0x8F,0xA2,0x61,0xD7,0x68,0xE9,0xBC,0xCA,0x22,0xCC,0xDB,0x64,0xCD,0xA9,0x92,0x2A,0x37,0xB3,0xCD,0xB7,0x3A,0x55,0x52,0x19,0xAE,0xFA,0x7A,0x57,0xA3,0xCE,0xC2,0xA3,0x42,0x57,0xD7,0x06,0x35,0x22,0xB8,0x8B,0x88,0x6C,0x70,0xC3,0x36,0xEE,0x62,0xEA,0xC6,0x75,0xCD,0x8B,0xBB,0xB9,0x6D,0xDF,0xFD,0x09};

Adafruit_CPlay_NeoPixel strip = Adafruit_CPlay_NeoPixel(NUM_PIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t samples = 64;
const double sampling = 100; //the speed at which we take samples, 10ms

const uint16_t N = 6; // Window size for moving average
float vReal[samples] = {0}; // A buffer to hold the real values
float vImag[samples] = {0}; // A buffer to hold the imaginary values
float freqDataPoints[100];
float MagnitudeDataPoints[100];
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
  Serial.begin(9600);
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
    // everywhere else where there isn't gravity
    else if(freqDataPoints[i] > 2)
    {
      float area = findArea(freqDataPoints[i], freqDataPoints[i+1], MagnitudeDataPoints[i], MagnitudeDataPoints[i+1]);
      sumPower += area;
    }
  }
}

void dataProcess(float *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    abscissa = ((i * 1.0 * sampling) / samples);

    // Serial.print(abscissa, 6);  
    // Serial.print(" Hz");
    // Serial.print(" ");
    // Serial.print(">Magnitude:");
    // Serial.println(vData[i], 4);

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
  //either adds moving average or real time value to the FFT array
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
  
  if (counter == samples - 1) 
  {
    cli(); // Disable interrupts
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Weigh data
    FFT.compute(FFTDirection::Forward); // Compute FFT
    FFT.complexToMagnitude(); // Compute magnitudes

    //Serial.println("Computed magnitudes:");
    dataProcess(vReal, (samples >> 1));

    calcPower();

    counter = 0;

    //incremenets number of times area has been calculated
    processCounter++;
    sei(); // Enable interrupts
  }


  counter++;
}

void loop() {
  CircuitPlayground.clearPixels();

  //every ten cycles of finding total power(minus gravity) and power under the region of interest
  if(processCounter == 15)
  {
    cli();
    sumPower = sumPower/processCounter;
    sumPeakPower = sumPeakPower/processCounter;
    Serial.print("Average whole Power: ");
    Serial.println(sumPower);
    Serial.print("Average Peak Power: ");
    Serial.println(sumPeakPower);

    //if the peak power is greater than 70% of the total power, then we consider it as a detection
    if((.7*(sumPower)) <= (sumPeakPower))
    {
      //do something
      CircuitPlayground.setPixelColor(5, 0x008000);
      delay(20000);
    }
    else
    {
      //red light	
      CircuitPlayground.setPixelColor(5, 0xFF0000);
      delay(20000);
    }

    sumPower = 0;
    sumPeakPower = 0;
    processCounter = 0;
    CircuitPlayground.clearPixels();
    sei();
  }
  
}
