// ===================================
// PLANT BIODATA SENSOR
// Interactive Projection Mapping
// ===================================

#include <Bounce2.h>
#include <EEPROMex.h>
#include <LEDFader.h>

int maxBrightness = 190;
const int scaleCount = 5;
const int scaleLen = 13;
int currScale = 0;
int scale[scaleCount][scaleLen] = {
    {12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}, // Chromatic
    {7, 1, 3, 5, 6, 8, 10, 12},                  // Major
    {7, 1, 3, 4, 6, 8, 9, 11},                   // DiaMinor
    {7, 1, 2, 2, 5, 6, 9, 11},                   // Indian
    {7, 1, 3, 4, 6, 8, 9, 11}                    // Minor
};

int root = 0;

const byte interruptPin = INT1;
const byte knobPin = A0;
Bounce button = Bounce();
const byte buttonPin = A1;
int menus = 5;
int mode = 0;
int currMenu = 0;
int pulseRate = 350;

const byte samplesize = 10;
const byte analysize = samplesize - 1;

const byte polyphony = 5;
int channel = 1;
int noteMin = 36;
int noteMax = 96;
byte QY8 = 0;
byte controlNumber = 80;
byte controlVoltage = 1;
long batteryLimit = 3000;
byte checkBat = 1;

byte timeout = 0;
int value = 0;
int prevValue = 0;

volatile unsigned long microseconds;
volatile byte index = 0;
volatile unsigned long samples[samplesize];

float threshold = 2.5;

float threshMin = 1.61;
float threshMax = 3.71;
float knobMin = 1;
float knobMax = 1024;

unsigned long previousMillis = 0;
unsigned long currentMillis = 1;
unsigned long lastPrintTime = 0;
unsigned long batteryCheck = 0;
unsigned long menuTimeout = 5000;

#define LED_NUM 6

LEDFader leds[LED_NUM] = {LEDFader(3), LEDFader(6), LEDFader(10),
                          LEDFader(5), LEDFader(9), LEDFader(11)};
int ledNums[LED_NUM] = {3, 6, 10, 5, 9, 11};
byte controlLED = 5;
byte noteLEDs = 1;

typedef struct _MIDImessage {
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
} MIDImessage;
MIDImessage noteArray[polyphony];
int noteIndex = 0;
MIDImessage controlMessage;

void setup() {
  pinMode(knobPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  button.attach(buttonPin);
  button.interval(5);

  randomSeed(analogRead(0));
  Serial.begin(115200);

  controlMessage.value = 0;
  if (noteLEDs)
    bootLightshow();
  attachInterrupt(interruptPin, sample, RISING);
}

void loop() {
  currentMillis = millis();

  checkButton();

  if (index >= samplesize) {
    analyzeSample();
  }
  checkNote();
  checkControl();
  checkLED();

  if (currMenu > 0)
    checkMenu();
}

void setNote(int value, int velocity, long duration, int notechannel) {
  for (int i = 0; i < polyphony; i++) {
    if (!noteArray[i].velocity) {
      noteArray[i].type = 0;
      noteArray[i].value = value;
      noteArray[i].velocity = velocity;
      noteArray[i].duration = currentMillis + duration;
      noteArray[i].channel = notechannel;

      if (QY8) {
        midiSerial(144, notechannel, value, velocity);
      } else {
        midiSerial(144, channel, value, velocity);
      }

      if (noteLEDs == 1) {
        for (byte j = 0; j < (LED_NUM - 1); j++) {
          if (!leds[j].is_fading()) {
            rampUp(i, maxBrightness, duration);
            break;
          }
        }
      } else if (noteLEDs == 2) {
        for (byte j = 1; j < (LED_NUM - 1); j++) {
          if (!leds[j].is_fading()) {
            rampUp(i, maxBrightness, duration);
            break;
          }
        }
      }

      break;
    }
  }
}

void setControl(int type, int value, int velocity, long duration) {
  controlMessage.type = type;
  controlMessage.value = value;
  controlMessage.velocity = velocity;
  controlMessage.period = duration;
  controlMessage.duration = currentMillis + duration;
}

void checkControl() {
  signed int distance = controlMessage.velocity - controlMessage.value;
  if (distance != 0) {
    if (currentMillis > controlMessage.duration) {
      controlMessage.duration = currentMillis + controlMessage.period;
      if (distance > 0) {
        controlMessage.value += 1;
      } else {
        controlMessage.value -= 1;
      }

      midiSerial(176, channel, controlMessage.type, controlMessage.value);

      if (controlVoltage) {
        if (distance > 0) {
          rampUp(controlLED, map(controlMessage.value, 0, 127, 0, 255), 5);
        } else {
          rampDown(controlLED, map(controlMessage.value, 0, 127, 0, 255), 5);
        }
      }
    }
  }
}

void checkNote() {
  for (int i = 0; i < polyphony; i++) {
    if (noteArray[i].velocity) {
      if (noteArray[i].duration <= currentMillis) {
        if (QY8) {
          midiSerial(144, noteArray[i].channel, noteArray[i].value, 0);
        } else {
          midiSerial(144, channel, noteArray[i].value, 0);
        }
        noteArray[i].velocity = 0;
        if (noteLEDs == 1)
          rampDown(i, 0, 225);
        if (noteLEDs == 2)
          rampDown(i + 1, 0, 225);
      }
    }
  }
}

void MIDIpanic() {
  for (byte i = 1; i < 128; i++) {
    delay(1);
    midiSerial(144, channel, i, 0);

    if (QY8) {
      for (byte k = 1; k < 5; k++) {
        delay(1);
        midiSerial(144, k, i, 0);
      }
    }
  }
}

void midiSerial(int type, int channel, int data1, int data2) {
  cli();
  data1 &= 0x7F;
  data2 &= 0x7F;
  byte statusbyte = (type | ((channel - 1) & 0x0F));
  sei();
}

void knobMode() {
}

void rampUp(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  led->fade(map(value, 0, 255, 0, maxBrightness), time);
}

void rampDown(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  led->fade(value, time);
}

void checkLED() {
  for (byte i = 0; i < LED_NUM; i++) {
    LEDFader *led = &leds[i];
    led->update();
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

long readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result;
  return result;
}

void checkBattery() {
  if (batteryCheck < currentMillis) {
    batteryCheck = currentMillis + 10000;

    if (readVcc() < batteryLimit) {
      if (checkBat) {
        for (byte j = 0; j < LED_NUM; j++) {
          leds[j].stop_fade();
          leds[j].set_value(0);
        }
        noteLEDs = 0;
        checkBat = 0;
      } else {
      }
    }
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

float mapfloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
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
    checkNote();
    checkControl();

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
  int prevScale = 0;
  while (runMode) {
    currScale = analogRead(knobPin);
    currScale = map(currScale, knobMin, knobMax, 0, scaleCount);

    pulse(value, maxBrightness, (pulseRate / 2));
    pulse(currScale, maxBrightness, (pulseRate / 4));

    if (currScale != prevScale) {
      leds[prevScale].stop_fade();
      leds[prevScale].set_value(0);
    }
    prevScale = currScale;

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }
    checkNote();
    checkControl();

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  }
  currMenu = 0;
  noteLEDs = 1;
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
  leds[currScale].stop_fade();
  leds[currScale].set_value(0);
}

void channelMode() {
  int runMode = 1;
  while (runMode) {
    channel = analogRead(knobPin);
    channel = map(channel, knobMin, knobMax, 1, 17);

    pulse(value, maxBrightness, (pulseRate / 4));

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    }
    checkNote();
    checkControl();

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
    checkNote();
    checkControl();

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

void sample() {
  if (index < samplesize) {
    samples[index] = micros() - microseconds;
    microseconds = samples[index] + microseconds;
    index += 1;
  }
}

void processChange(unsigned long delta, unsigned long averg, byte change) {
  if (change) {
    int dur = 150 + (map(delta % 127, 1, 127, 100, 2500));
    int ramp = 3 + (dur % 100);
    int notechannel = random(1, 5);

    if (noteLEDs > 0) {
      rampUp(random(0, LED_NUM), 255, 50);
    }

    int setnote = map(averg % 127, 1, 127, noteMin, noteMax);
    setnote = scaleNote(setnote, scale[currScale], root);

    if (QY8) {
      setNote(setnote, 100, dur, notechannel);
    } else {
      setNote(setnote, 100, dur, channel);
    }

    setControl(controlNumber, controlMessage.value, delta % 127, ramp);
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
    unsigned long currentTime = millis();

    if ((currentTime - lastPrintTime) > 100) {
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

      lastPrintTime = currentTime;
    }

    index = 0;
  }
}

int scaleSearch(int note, int scale[], int scalesize) {
  for (byte i = 1; i < scalesize; i++) {
    if (note == scale[i]) {
      return note;
    } else {
      if (note < scale[i]) {
        return scale[i];
      }
    }
  }
  return 6;
}

int scaleNote(int note, int scale[], int root) {
  int scaled = note % 12;
  int octave = note / 12;
  int scalesize = (scale[0]);
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + root;
  return scaled;
}