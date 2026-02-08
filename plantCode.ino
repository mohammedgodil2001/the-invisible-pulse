#include <Bounce2.h> // Bounce2 removes noise from button presses so one press is detected as one clean action.
#include <EEPROMex.h> // EEPROMex stores data permanently on the Arduino, so the settings are not lost when the power is turned off.
#include <LEDFader.h> // LEDFader allows LEDs to fade in and out smoothly without using delays or harsh ON/OFF switching.

int maxBrightness = 190;
const int scaleCount = 5;
const int scaleLen = 13; 
int currScale = 0;       
int scale[scaleCount][scaleLen] = {
    {12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}, 
    {7, 1, 3, 5, 6, 8, 10, 12},                  
    {7, 1, 3, 4, 6, 8, 9, 11},                   
    {7, 1, 2, 2, 5, 6, 9, 11},                   
    {7, 1, 3, 4, 6, 8, 9, 11}                    
};

int root = 0; 
const byte interruptPin = INT1;  
Bounce button = Bounce();  
const byte buttonPin = A1; 
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
volatile unsigned long microseconds; 
volatile byte index = 0;
volatile unsigned long samples[samplesize];
float threshold = 2.5; 
float threshMin = 1.61; 
float threshMax = 3.71; 
unsigned long previousMillis = 0;
unsigned long currentMillis = 1;
unsigned long lastPrintTime = 0;  
unsigned long batteryCheck = 0;   
#define LED_NUM 6

LEDFader leds[LED_NUM] = {LEDFader(3),  // Red
                          LEDFader(6),  // Green
                          LEDFader(10), // White
                          LEDFader(5),  LEDFader(9), LEDFader(11)};
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
MIDImessage
    controlMessage; 

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  button.attach(buttonPin); // we are attaching the button to this library to make it less noisy
  button.interval(5);
  randomSeed(analogRead(0)); 
  Serial.begin(115200); 
  controlMessage.value = 0; // it is for MIDI
  if (noteLEDs)
    bootLightshow(); // this function is for all the 5 leds to turn on when the arduino is connected to the computer
  attachInterrupt(interruptPin, sample, RISING); // bcs of this arduino can catch all the signals from the 555 timer
}

void loop() {
  currentMillis = millis(); 
  if (index >= samplesize) {
    analyzeSample();
  } 
  checkNote();    
  checkControl(); 
  checkLED();     
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
        for (byte j = 1; j < (LED_NUM - 1);
             j++) { 
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
  controlMessage.duration =
      currentMillis + duration; 
}

void checkControl() {
  signed int distance = controlMessage.velocity - controlMessage.value;
  if (distance != 0) {
    if (currentMillis > controlMessage.duration) { // and duration expired
      controlMessage.duration =
          currentMillis + controlMessage.period; // extend duration
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
  // 120 - all sound off
  // 123 - All Notes off
  // midiSerial(21, panicChannel, 123, 0); //123 kill all notes

  // brute force all notes Off
  for (byte i = 1; i < 128; i++) {
    delay(1);                       // don't choke on note offs!
    midiSerial(144, channel, i, 0); // clear notes on main channel

    if (QY8) { // clear on all four channels
      for (byte k = 1; k < 5; k++) {
        delay(1); // don't choke on note offs!
        midiSerial(144, k, i, 0);
      }
    }
  }
}


static int lastNote = -1;
static int lastVelocity = -1;

void midiSerial(int type, int channel, int data1, int data2) {
    if (type == 144 || type == 128) {
    
    data1 &= 0x7F;  
    data2 &= 0x7F;  
    
    if (data1 != lastNote || data2 != lastVelocity) {
      
      Serial.print("M,");
      Serial.print(data1);      // Note number
      Serial.print(",");
      Serial.print(data2);      // Velocity
      Serial.print(",");
      Serial.println(channel);  // Channel
      

      lastNote = data1;
      lastVelocity = data2;
    }
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

void checkLED() {
  for (byte i = 0; i < LED_NUM; i++) {
    LEDFader *led = &leds[i];
    led->update();
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
  for (byte i = 5; i > 0; i--) { // so basically we are running a for loop and going over every loop like 5,4,3,2,1 and than turning their brightness to 200 for 0.15 seconds and than checking while it is fading run checkLED and one by one fade every led faster
    LEDFader *led = &leds[i - 1];
    led->fade(200, 150); // fade up
    while (led->is_fading())
      checkLED();

    led->fade(0, 150 + i * 17); // fade down
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

void sample() {
  if (index < samplesize) {
    samples[index] = micros() - microseconds;
    microseconds =
        samples[index] + microseconds; 
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

  // FSM State Variables
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

    const float TOUCH_OFFSET = 25.0; // 45 , 15
    const float REL_OFFSET = 15.0;  // 30 , 10

    if (!isTouched) {
      baseline = (baseline * 0.95) + ((float)delta * 0.05);

      if (delta > (baseline + TOUCH_OFFSET)) {
        isTouched = true;
        touchStartTime = millis();
        processChange(delta, averg, 1);
      }
    } else {


      if (millis() - touchStartTime > 10000) {
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

    

    static unsigned long lastMidiSend = 0;
    if (millis() - lastMidiSend > 200) {
      
      int setnote = map(delta, 0, 500, noteMin, noteMax);
      setnote = scaleNote(setnote, scale[currScale], root);
      int velocity = map(delta, 0, 500, 60, 127);
      
      byte tempLED = noteLEDs;
      noteLEDs = 0;
      setNote(setnote, velocity, 400, channel);
      noteLEDs = tempLED;
      
      lastMidiSend = millis();
    }
 

    Serial.print("D,");
    Serial.println(delta);



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