// ===================================
// PLANT BIODATA SENSOR
// Interactive Projection Mapping
// ===================================

#include <Bounce2.h>
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

// Button and Menu Variables
Bounce button = Bounce();
int currMenu = 0;
int menus = 5;
int value = 0;
int prevValue = 0;
int pulseRate = 350;
unsigned long menuTimeout = 5000;

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
float knobMin = 1;
float knobMax = 1024;

void setup() {
  pinMode(knobPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  button.attach(buttonPin);
  button.interval(5);
  
  randomSeed(analogRead(0));
  
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
  
  checkButton();
  
  if (index >= samplesize) {
    analyzeSample();
  }
  
  checkLED();
  
  if (currMenu > 0)
    checkMenu();
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

    if (baseline == 0) {
      baseline = delta;
    }

    const float TOUCH_OFFSET = 45.0;
    const float REL_OFFSET = 30.0;

    if (!isTouched) {
      baseline = (baseline * 0.95) + ((float)delta * 0.05);

      if (delta > (baseline + TOUCH_OFFSET)) {
        isTouched = true;
        touchStartTime = millis();
        processChange(delta, averg, 1);
      }
    } else {
      if (millis() - touchStartTime > 10000) {
        Serial.println("!!! AUTO-RESET (Stuck Protection) !!!");
        baseline = delta;
        isTouched = false;
        if (noteLEDs && LED_NUM > 0) {
          rampUp(0, 0, 0);
        }
      }
      else if (delta < (baseline + REL_OFFSET)) {
        isTouched = false;
      }
    }

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

void checkButton() {
  button.update();
  if (button.fell()) {
    noteLEDs = 0;
    for (byte j = 0; j < LED_NUM; j++) {
      leds[j].stop_fade();
      leds[j].set_value(0);
    }

    switch (currMenu) {
    case 0:
      prevValue = 0;
      currMenu = 1;
      previousMillis = currentMillis;
      break;
    case 1:
      switch (value) {
      case 0:
        thresholdMode();
        return;
        break;
      case 1:
        scaleMode();
        return;
        break;
      case 2:
        channelMode();
        return;
        break;
      case 3:
        brightnessMode();
        return;
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }
  }
}

void checkMenu() {
  static unsigned long debugTimer = 0;

  value = analogRead(knobPin);
  int rawValue = value;

  value = map(value, knobMin, knobMax, 0, menus);

  if (millis() - debugTimer > 500) {
    debugTimer = millis();
  }

  if (value != prevValue) {
    leds[prevValue].stop_fade();
    leds[prevValue].set_value(0);
    prevValue = value;
    previousMillis = currentMillis;
  }

  switch (currMenu) {
  case 1:
    noteLEDs = 0;
    pulse(value, maxBrightness, pulseRate);
    break;
  case 2:
    noteLEDs = 0;
    pulse(value, maxBrightness, (pulseRate / 2));
    break;
  }

  if ((currentMillis - previousMillis) > menuTimeout) {
    currMenu = 0;
    if (maxBrightness > 1)
      noteLEDs = 1;
    leds[prevValue].stop_fade();
    leds[prevValue].set_value(0);
  }
}

void pulse(int ledPin, int maxValue, int time) {
  LEDFader *led = &leds[ledPin];
  if (led->is_fading() == false) {
    if (led->get_value() > 0) {
      led->fade(0, time);
    } else
      led->fade(maxValue, time);
  }
}

void thresholdMode() {
  int runMode = 1;
  noteLEDs = 2;
  while (runMode) {
    threshold = analogRead(knobPin);
    threshold = mapfloat(threshold, knobMin, knobMax, threshMin, threshMax);
    pulse(value, maxBrightness, (pulseRate / 2));

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  }
  currMenu = 0;
  noteLEDs = 1;
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

void scaleMode() {
  int runMode = 1;
  while (runMode) {
    pulse(value, maxBrightness, (pulseRate / 2));

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  }
  currMenu = 0;
  noteLEDs = 1;
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

void channelMode() {
  int runMode = 1;
  while (runMode) {
    pulse(value, maxBrightness, (pulseRate / 4));

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  }
  currMenu = 0;
  noteLEDs = 1;
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

void brightnessMode() {
  int runMode = 1;
  while (runMode) {
    maxBrightness = analogRead(knobPin);
    maxBrightness = map(maxBrightness, knobMin, knobMax, 1, 255);

    if (maxBrightness > 1)
      pulse(value, maxBrightness, (pulseRate / 2));
    else
      pulse(value, 1, (pulseRate / 6));

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  }
  currMenu = 0;
  if (maxBrightness > 1)
    noteLEDs = 1;
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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