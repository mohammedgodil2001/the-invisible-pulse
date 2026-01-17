// ===================================
// PLANT BIODATA SENSOR
// Interactive Projection Mapping
// ===================================

#include <LEDFader.h>

// Pin Definitions
const byte interruptPin = INT1;  // Galvanometer input (555 timer)
const byte knobPin = A0;         // Potentiometer analog input
const byte buttonPin = A1;       // Tactile button input

// LED Pin Array
#define LED_NUM 6
LEDFader leds[LED_NUM] = {LEDFader(3),  // Red
                          LEDFader(6),  // Green
                          LEDFader(10), // White
                          LEDFader(5),  
                          LEDFader(9),  
                          LEDFader(11)};
int ledNums[LED_NUM] = {3, 6, 10, 5, 9, 11};
int maxBrightness = 190;
byte noteLEDs = 1;

// Timing Variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 1;

// Sensor Sampling Variables
const byte samplesize = 10;            // Sample array size
const byte analysize = samplesize - 1; // Trim for analysis array
volatile unsigned long microseconds;   // Sampling timer
volatile byte index = 0;               // Current sample index
volatile unsigned long samples[samplesize]; // Store time between pulses

// Threshold Variables
float threshold = 2.5;
float threshMin = 1.61;
float threshMax = 3.71;

void setup() {
  pinMode(knobPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  
  Serial.println("Plant Biodata Sensor Initialized");
  
  if (noteLEDs)
    bootLightshow();
  
  Serial.println("Waiting for sensor input...");
  
  attachInterrupt(interruptPin, sample, RISING);
  Serial.println("Interrupt attached - ready to capture pulses");
}

void loop() {
  currentMillis = millis();
  
  if (index >= samplesize) {
    analyzeSample();
  }
  
  checkLED();
}

void sample() {
  if (index < samplesize) {
    samples[index] = micros() - microseconds;
    microseconds = samples[index] + microseconds;
    index += 1;
  }
}

void processChange(unsigned long delta, unsigned long averg, byte change) {
  if (change) {
    if (noteLEDs > 0) {
      rampUp(random(0, LED_NUM), 255, 50);
    }
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║            TOUCH DETECTED!            ║");
    Serial.println("╚═══════════════════════════════════════╝");
  }
}

void analyzeSample() {
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;

  static float baseline = 0.0;
  static bool isTouched = false;
  static unsigned long touchStartTime = 0;

  if (index == samplesize) {
    unsigned long sampanalysis[analysize];
    
    for (byte i = 0; i < analysize; i++) {
      sampanalysis[i] = samples[i + 1];
      
      if (sampanalysis[i] > maxim) {
        maxim = sampanalysis[i];
      }
      if (sampanalysis[i] < minim) {
        minim = sampanalysis[i];
      }
      
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];
    }

    averg = averg / analysize;
    stdevi = sqrt(stdevi / analysize - (float)averg * (float)averg);
    
    if (stdevi < 1) {
      stdevi = 1.0;
    }
    
    delta = maxim - minim;

    // Initialize baseline on startup
    if (baseline == 0) {
      baseline = delta;
    }

    // Hysteresis thresholds
    const float TOUCH_OFFSET = 45.0;
    const float REL_OFFSET = 30.0;

    if (!isTouched) {
      // Drift compensation
      baseline = (baseline * 0.95) + ((float)delta * 0.05);

      // Touch detection
      if (delta > (baseline + TOUCH_OFFSET)) {
        isTouched = true;
        touchStartTime = millis();
        processChange(delta, averg, 1);
      }
    } else {
      // Stuck protection
      if (millis() - touchStartTime > 10000) {
        Serial.println("!!! AUTO-RESET (Stuck Protection) !!!");
        baseline = delta;
        isTouched = false;
        if (noteLEDs && LED_NUM > 0) {
          rampUp(0, 0, 0);
        }
      }
      // Release detection
      else if (delta < (baseline + REL_OFFSET)) {
        isTouched = false;
      }
    }

    // Status monitoring
    static unsigned long lastPrintTime = 0;
    if ((millis() - lastPrintTime) > 100) {
      if (isTouched) {
        Serial.println("╔═══════════════════════════════════════╗");
        Serial.println("║            TOUCHED!                   ║");
        Serial.println("╚═══════════════════════════════════════╝");
      }
      Serial.print("Delta: ");
      Serial.print(delta);
      Serial.print(" | Base: ");
      Serial.print((int)baseline);
      Serial.print(" | Thresh: >");
      if (isTouched)
        Serial.println((int)(baseline + REL_OFFSET));
      else
        Serial.println((int)(baseline + TOUCH_OFFSET));

      lastPrintTime = millis();
    }

    index = 0;
  }
}

void checkLED() {
  for (byte i = 0; i < LED_NUM; i++) {
    LEDFader *led = &leds[i];
    led->update();
  }
}

void rampUp(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  led->fade(map(value, 0, 255, 0, maxBrightness), time);
}

void rampDown(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  led->fade(value, time);
}

void bootLightshow() {
  for (byte i = 5; i > 0; i--) {
    LEDFader *led = &leds[i - 1];
    
    led->fade(200, 150);
    while (led->is_fading())
      checkLED();

    led->fade(0, 150 + i * 17);
    while (led->is_fading())
      checkLED();
  }
}