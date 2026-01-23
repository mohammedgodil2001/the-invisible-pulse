#include <Bounce2.h> // Bounce2 removes noise from button presses so one press is detected as one clean action.
#include <EEPROMex.h> // EEPROMex stores data permanently on the Arduino, so the settings are not lost when the power is turned off.
#include <LEDFader.h> // LEDFader allows LEDs to fade in and out smoothly without using delays or harsh ON/OFF switching.

int maxBrightness = 190;
const int scaleCount = 5;
const int scaleLen = 13; // maximum scale length plus 1 for 'used length'
int currScale = 0;       // current scale, default Chrom
int scale[scaleCount][scaleLen] = {
    {12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}, // Chromatic
    {7, 1, 3, 5, 6, 8, 10, 12},                  // Major
    {7, 1, 3, 4, 6, 8, 9, 11},                   // DiaMinor
    {7, 1, 2, 2, 5, 6, 9, 11},                   // Indian
    {7, 1, 3, 4, 6, 8, 9, 11}                    // Minor
};

int root = 0; // initialize for root, pitch shifting


const byte interruptPin = INT1; // galvanometer input
// const byte interruptPin = INT0; //galvanometer input
const byte knobPin = A0;   // knob analog input
Bounce button = Bounce();  // debounce button using Bounce2
const byte buttonPin = A1; // tact button input
int menus = 5;             // number of main menus
int mode = 0;              // 0 = Threshold, 1 = Scale, 2 = Brightness
int currMenu = 0;
int pulseRate = 350; // base pulse rate

const byte samplesize = 10;            // set sample array size
const byte analysize = samplesize - 1; // trim for analysis array

const byte polyphony = 5; // above 8 notes may run out of ram
int channel = 1; // setting channel to 11 or 12 often helps simply computer midi
                 // routing setups
int noteMin = 36; // C2  - keyboard note minimum
int noteMax = 96; // C7  - keyboard note maximum
byte QY8 = 0; // sends each note out chan 1-4, for use with General MIDI like
              // Yamaha QY8 sequencer
byte controlNumber = 80; // set to mappable control, low values may interfere
                         // with other soft synth controls!!
byte controlVoltage =
    1; // output PWM CV on controlLED, pin 17, PB3, digital 11 *lowpass filter
long batteryLimit = 3000; // voltage check minimum, 3.0~2.7V under load; causes
                          // lightshow to turn off (save power)
byte checkBat = 1;

byte timeout = 0;
int value = 0;
int prevValue = 0;

volatile unsigned long microseconds; // sampling timer
volatile byte index = 0;
volatile unsigned long samples[samplesize];

float threshold = 2.5; // Adjusted for lower noise floor

float threshMin = 1.61; // scaling threshold min
float threshMax = 3.71; // scaling threshold max
float knobMin = 1;
float knobMax = 1024;

unsigned long previousMillis = 0;
unsigned long currentMillis = 1;
unsigned long lastPrintTime = 0;  // Timer for slow idle printing
unsigned long batteryCheck = 0;   // battery check delay timer
unsigned long menuTimeout = 5000; // 5 seconds timeout in menu mode

#define LED_NUM 6

LEDFader leds[LED_NUM] = {LEDFader(3),  // Red
                          LEDFader(6),  // Green
                          LEDFader(10), // White
                          LEDFader(5),  LEDFader(9), LEDFader(11)};
int ledNums[LED_NUM] = {3, 6, 10, 5, 9, 11};
byte controlLED = 5; // array index of control LED (CV out)
byte noteLEDs = 1;   // performs lightshow set at noteOn event

typedef struct _MIDImessage { // build structure for Note and Control
                              // MIDImessages
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
} MIDImessage;
MIDImessage noteArray[polyphony]; // manage MIDImessage data as an array with
                                  // size polyphony
int noteIndex = 0;
MIDImessage
    controlMessage; // manage MIDImessage data for Control Message (CV out)

void setup() {
  pinMode(knobPin, INPUT); // here we are setting up the knb and button to take theier vales and read it later
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
  // checkBattery(); //on low power, shutoff lightShow, continue MIDI operation

  checkButton(); // its about to get really funky in here

  if (index >= samplesize) {
    analyzeSample();
  } // if samples array full, also checked in analyzeSample(), call sample
    // analysis
  checkNote();    // turn off expired notes
  checkControl(); // update control value
  checkLED();     // LED management without delay()

  if (currMenu > 0)
    checkMenu(); // allow main loop by checking current menu mode, and updating
                 // millis
}

void setNote(int value, int velocity, long duration, int notechannel) {
  // find available note in array (velocity = 0);
  for (int i = 0; i < polyphony; i++) {
    if (!noteArray[i].velocity) {
      // if velocity is 0, replace note in array
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

      if (noteLEDs == 1) {                         // normal mode
        for (byte j = 0; j < (LED_NUM - 1); j++) { // find available LED and set
          if (!leds[j].is_fading()) {
            rampUp(i, maxBrightness, duration);
            break;
          }
        }
      } else if (noteLEDs == 2) { // threshold special display mode
        for (byte j = 1; j < (LED_NUM - 1);
             j++) { // find available LED above first and set
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
      currentMillis + duration; // schedule for update cycle
}

void checkControl() {
  // need to make this a smooth slide transition, using high precision
  // distance is current minus goal
  signed int distance = controlMessage.velocity - controlMessage.value;
  // if still sliding
  if (distance != 0) {
    // check timing
    if (currentMillis > controlMessage.duration) { // and duration expired
      controlMessage.duration =
          currentMillis + controlMessage.period; // extend duration
      // update value
      if (distance > 0) {
        controlMessage.value += 1;
      } else {
        controlMessage.value -= 1;
      }

      // send MIDI control message after ramp duration expires, on each
      // increment
      midiSerial(176, channel, controlMessage.type, controlMessage.value);

      // send out control voltage message on pin 17, PB3, digital 11
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
        // send noteOff for all notes with expired duration
        if (QY8) {
          midiSerial(144, noteArray[i].channel, noteArray[i].value, 0);
        } else {
          midiSerial(144, channel, noteArray[i].value, 0);
        }
        noteArray[i].velocity = 0;
        if (noteLEDs == 1)
          rampDown(i, 0, 225);
        if (noteLEDs == 2)
          rampDown(i + 1, 0, 225); // special threshold display mode
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

// void midiSerial(int type, int channel, int data1, int data2) {

//   cli(); // kill interrupts, probably unnessisary
//   //  Note type = 144
//   //  Control type = 176
//   // remove MSBs on data
//   data1 &= 0x7F; // number
//   data2 &= 0x7F; // velocity

//   byte statusbyte = (type | ((channel - 1) & 0x0F));

//   // Serial.write(statusbyte);
//   // Serial.write(data1);
//   // Serial.write(data2);
//   sei(); // enable interrupts
// }


static int lastNote = -1;
static int lastVelocity = -1;

void midiSerial(int type, int channel, int data1, int data2) {
  // Only process Note On/Off messages (144)
  // if (type == 144) {
    if (type == 144 || type == 128) {
    
    // Force valid MIDI range
    data1 &= 0x7F;  // Note number 0-127
    data2 &= 0x7F;  // Velocity 0-127
    
    // Only send if note OR velocity actually changed
    if (data1 != lastNote || data2 != lastVelocity) {
      
      Serial.print("M,");
      Serial.print(data1);      // Note number
      Serial.print(",");
      Serial.print(data2);      // Velocity
      Serial.print(",");
      Serial.println(channel);  // Channel
      
      // Remember last values
      lastNote = data1;
      lastVelocity = data2;
    }
  }
}









void knobMode() {
  // scroll through menus and select values using only a single knob
  // keep dreamin' kid,
}

void rampUp(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  // scale the value parameter against a new maxBrightness global variable
  //   led->fade(value, time);
  led->fade(map(value, 0, 255, 0, maxBrightness), time);
}

void rampDown(int ledPin, int value, int time) {
  LEDFader *led = &leds[ledPin];
  // led->set_value(255); //turn on
  led->fade(value, time); // fade out
}

void checkLED() {
  // iterate through LED array and call update
  for (byte i = 0; i < LED_NUM; i++) {
    LEDFader *led = &leds[i];
    led->update();
  }
}

/*
Press the button: When you press the button, currMenu is set to 1, which means you enter the menu mode.

Read the knob and map the value: The code reads the knob’s position and maps that value to a menu option (for example, from 0 to 5 if you have six menu items).

Turn off note LEDs: Any LEDs that were used for notes or other interactions are turned off so that only the menu-related LEDs are active.

Pulse the menu LED: The LED that represents the currently selected menu item will start to pulse on and off gently, so you can see which menu option you’re on.
*/


/* * MENU & BUTTON LOGIC SUMMARY
 * * 1. DEFAULT STATE (Play Mode):
 * - currMenu = 0.
 * - checkButton() runs constantly in void loop() but only listens.
 * - checkMenu() is SKIPPED because (currMenu > 0) is false.
 * - System focuses purely on plant data input.
 * * 2. TRIGGER (Button Press 1):
 * - checkButton() detects press -> switches currMenu to 1.
 * - Main loop detects (currMenu > 0) -> activates checkMenu().
 * * 3. MENU MODE (currMenu = 1):
 * - checkMenu() runs continuously.
 * - Reads Potentiometer -> Maps value to menu options (0-3).
 * - Flashes LEDs to indicate selection.
 * * 4. SELECTION (Button Press 2):
 * - checkButton() detects press while in Menu Mode.
 * - Enters specific sub-menu (Threshold, Scale, etc.) based on knob position.
 */

// so this fucntion basically means that when i press the button for the first time currMenu is by default 0 and if i press the button it is 0 and arduino makes it value to 1 and it is 1 and if I press the button again and turn the knob on it runs thresholdMode bcs value os 0
void checkButton() {
  button.update(); // this filters out electrical noise and it takes this function from the Bounce2 library
  if (button.fell()) { // fell means when the button was pressed
    // Serial.println("BUTTON PRESSED "); // for debugging

    noteLEDs = 0; // turning off all the leds when the button is pressed. 
    for (byte j = 0; j < LED_NUM; j++) { // accessing all the leds in a loop 
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
  // DEBUG: Print status every 500ms so we don't spam the screen too fast
  static unsigned long debugTimer = 0; // unsigned long - hold a very bg number , static - it rememeber the last number whe the function was ran and do not reset to zero

  value = analogRead(knobPin); // currnet reading value from the knob pin
  int rawValue = value; // storing that pin value in a new variable

  // Scale knob value against number of menus
  value = map(value, knobMin, knobMax, 0, menus); // taking the value from the knob pin and mappin it between 0 to 5

  if (millis() - debugTimer > 500) { // millis function will give mili seconds from the time arduino has started  
    debugTimer = millis();
  }

  // Set LEDs to flash based on value
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
    // Serial.println("TIMEOUT! Exiting Menu..."); // DEBUG
    currMenu = 0;
    if (maxBrightness > 1)
      noteLEDs = 1;
    leds[prevValue].stop_fade();
    leds[prevValue].set_value(0);
  }
}

long readVcc() { // https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC))
    ;
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void checkBattery() {
  // check battery voltage against internal 1.1v reference
  // if below the minimum value, turn off the light show to save power
  // don't check on every loop, settle delay in readVcc() slows things down a
  // bit
  if (batteryCheck < currentMillis) {
    batteryCheck = currentMillis + 10000; // reset for next battery check

    if (readVcc() < batteryLimit) { // if voltage > valueV
      // battery failure
      if (checkBat) { // first battery failure
        for (byte j = 0; j < LED_NUM; j++) {
          leds[j].stop_fade();
          leds[j].set_value(0);
        } // reset leds, power savings
        noteLEDs = 0; // shut off lightshow set at noteOn event, power savings
        checkBat = 0; // update, first battery failure identified
      } else {        // not first low battery cycle
        // do nothing, lights off indicates low battery
        // MIDI continues to flow, MIDI data eventually garbles at very low
        // voltages some USB-MIDI interfaces may crash due to garbled data
      }
    }
  }
}

void pulse(int ledPin, int maxValue, int time) {
  LEDFader *led = &leds[ledPin];
  // check on the state of the LED and force it to pulse
  if (led->is_fading() == false) { // if not fading
    if (led->get_value() > 0) {    // and is illuminated
      led->fade(0, time);          // fade down
    } else
      led->fade(maxValue, time); // fade up
  }
}

void bootLightshow() {
  for (byte i = 5; i > 0; i--) { // so basically we are running a for loop and going over every loop like 5,4,3,2,1 and than turning their brightness to 200 for 0.15 seconds and than checking while it is fading run checkLED and one by one fade every led faster
    LEDFader *led = &leds[i - 1];
    //    led->set_value(200); //set to max

    led->fade(200, 150); // fade up
    while (led->is_fading())
      checkLED();

    led->fade(0, 150 + i * 17); // fade down
    while (led->is_fading())
      checkLED();
    // move to next LED
  }
}

// provide float map function
float mapfloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// debug SRAM memory size
int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
} // print free RAM at any point

void thresholdMode() {
  int runMode = 1;
  noteLEDs = 2; // turn on special Note visualization for feedback on threshold effect
  while (runMode) {
    // float knobValue
    threshold = analogRead(knobPin);
    // set threshold to knobValue mapping
    threshold = mapfloat(threshold, knobMin, knobMax, threshMin, threshMax);
    pulse(value, maxBrightness, (pulseRate / 2)); // pulse for current menu

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    } // keep samples running
    checkNote();    // turn off expired notes
    checkControl(); // update control value

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  } // after button press retain threshold setting
  currMenu = 0; // return to main program
  noteLEDs = 1; // normal light show
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

void scaleMode() {
  int runMode = 1;
  int prevScale = 0;
  while (runMode) {
    currScale = analogRead(knobPin);
    // set current Scale choice
    currScale = map(currScale, knobMin, knobMax, 0, scaleCount);

    pulse(value, maxBrightness, (pulseRate / 2)); // pulse for current menu
    pulse(currScale, maxBrightness,
          (pulseRate / 4)); // display selected scale if scaleCount <= 5

    if (currScale != prevScale) { // clear last value if change
      leds[prevScale].stop_fade();
      leds[prevScale].set_value(0);
    }
    prevScale = currScale;

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    } // keep samples running
    checkNote();    // turn off expired notes
    checkControl(); // update control value

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  } // after button press retain threshold setting
  currMenu = 0; // return to main program
  noteLEDs = 1; // normal light show
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
  leds[currScale].stop_fade();
  leds[currScale].set_value(0);
}

void channelMode() {
  int runMode = 1;
  while (runMode) {
    channel = analogRead(knobPin);
    // set current MIDI Channel between 1 and 16
    channel = map(channel, knobMin, knobMax, 1, 17);

    pulse(value, maxBrightness, (pulseRate / 4)); // pulse for current menu

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    } // keep samples running
    checkNote();    // turn off expired notes
    checkControl(); // update control value

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  } // after button press retain threshold setting
  currMenu = 0; // return to main program
  noteLEDs = 1; // normal light show
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

void brightnessMode() {
  int runMode = 1;
  while (runMode) {
    maxBrightness = analogRead(knobPin);
    // set led maxBrightness
    maxBrightness = map(maxBrightness, knobMin, knobMax, 1, 255);

    if (maxBrightness > 1)
      pulse(value, maxBrightness, (pulseRate / 2)); // pulse for current menu
    else
      pulse(value, 1, (pulseRate / 6)); // fast dim pulse for 0 note lightshow

    checkLED();
    if (index >= samplesize) {
      analyzeSample();
    } // keep samples running
    checkNote();    // turn off expired notes
    checkControl(); // update control value

    button.update();
    if (button.fell())
      runMode = 0;

    currentMillis = millis();
  } // after button press retain threshold setting
  currMenu = 0; // return to main program
  if (maxBrightness > 1)
    noteLEDs = 1; // normal light show, unles lowest value
  leds[prevValue].stop_fade();
  leds[prevValue].set_value(0);
}

// interrupt timing sample array
void sample() {
  if (index < samplesize) {
    samples[index] = micros() - microseconds;
    microseconds =
        samples[index] + microseconds; // rebuild micros() value w/o recalling
    // micros() is very slow
    // try a higher precision counter
    // samples[index] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    index += 1;
  }
}

// Helper to handle triggers (Sound + LED)
void processChange(unsigned long delta, unsigned long averg, byte change) {
  if (change) {
    int dur = 150 + (map(delta % 127, 1, 127, 100, 2500));
    int ramp = 3 + (dur % 100);
    int notechannel = random(1, 5);

    // Visual feedback
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


// original
// void analyzeSample() {
//   unsigned long averg = 0;
//   unsigned long maxim = 0;
//   unsigned long minim = 100000;
//   float stdevi = 0;
//   unsigned long delta = 0;

//   // FSM State Variables
//   static float baseline = 0.0;
//   static bool isTouched = false;
//   static unsigned long touchStartTime = 0;

//   if (index == samplesize) { // array is full
//     unsigned long sampanalysis[analysize];
//     for (byte i = 0; i < analysize; i++) {
//       sampanalysis[i] = samples[i + 1];
//       if (sampanalysis[i] > maxim) {
//         maxim = sampanalysis[i];
//       }
//       if (sampanalysis[i] < minim) {
//         minim = sampanalysis[i];
//       }
//       averg += sampanalysis[i];
//       stdevi += sampanalysis[i] * sampanalysis[i];
//     }

//     averg = averg / analysize;
//     stdevi = sqrt(stdevi / analysize - (float)averg * (float)averg);
//     if (stdevi < 1) {
//       stdevi = 1.0;
//     }
//     delta = maxim - minim;

    

//     // 1. Initialize Baseline instantly on startup
//     if (baseline == 0) {
//       baseline = delta;
//     }

//     const float TOUCH_OFFSET = 150.0; // 45
//     const float REL_OFFSET = 80.0;  // 30

//     // 2. Logic Handler
//     if (!isTouched) {
//       baseline = (baseline * 0.95) + ((float)delta * 0.05);

//       if (delta > (baseline + TOUCH_OFFSET)) {
//         isTouched = true;
//         touchStartTime = millis();
//         // Trigger LED/Sound
//         processChange(delta, averg, 1);
//       }
//     } else {


//       if (millis() - touchStartTime > 10000) {
//         baseline = delta; 
//         isTouched = false;
//         if (noteLEDs && LED_NUM > 0) {
//           rampUp(0, 0, 0);
//         }
//       }

      
//       else if (delta < (baseline + REL_OFFSET)) {
//         isTouched = false;
//       }
//     }

    
//     // static unsigned long lastPrintTime = 0;
//     // unsigned long currentTime = millis();
//     // if ((currentTime - lastPrintTime) > 100) {
//     //   if (isTouched) {
//     //     Serial.println("╔═══════════════════════════════════════╗");
//     //     Serial.println("║            TOUCHED!                   ║");
//     //     Serial.println("╚═══════════════════════════════════════╝");
//     //   }
//     //   Serial.print("Delta: ");
//     //   Serial.print(delta);
//     //   Serial.print(" | Base: ");
//     //   Serial.print((int)baseline);
//     //   Serial.print(" | Thresh: >");
//     //   if (isTouched)
//     //     Serial.println((int)(baseline + REL_OFFSET)); // Show Release point
//     //   else
//     //     Serial.println((int)(baseline + TOUCH_OFFSET)); // Show Touch point

//     //   lastPrintTime = currentTime;
//     // }


//     static unsigned long lastMidiSend = 0;
//     if (millis() - lastMidiSend > 50) {  // Adjust interval as needed
//       // processChange(delta, averg, 1);
//       lastMidiSend = millis();
//     }
    
    

    
    

//     Serial.print("D,");
//     Serial.println(delta);



   



//   //  static unsigned long lastSend = 0;
//   //   if (millis() - lastSend > 50) {  // Limit speed to every 50ms
       
//   //      // 1. VISUALS (Send "D" for TouchDesigner)
//   //      Serial.print("D,");
//   //      Serial.println(delta);

//   //      // 2. MUSIC (Calculate Perfect Note and send "P" for VCV Rack)
//   //      int myNote = map(averg % 127, 1, 127, noteMin, noteMax); 
//   //      myNote = scaleNote(myNote, scale[currScale], root); 

//   //      Serial.print("P,");
//   //      Serial.println(myNote);

//   //      // 3. LEDS (Keep flashing lights)
//   //     //  processChange(delta, averg, 1);
       
//   //      lastSend = millis();
//   //   }




//     index = 0;
//   }
// }



// calude - first 
// void analyzeSample() {
//   unsigned long averg = 0;
//   unsigned long maxim = 0;
//   unsigned long minim = 100000;
//   float stdevi = 0;
//   unsigned long delta = 0;

//   static float baseline = 0.0;
//   static bool isTouched = false;
//   static unsigned long touchStartTime = 0;

//   if (index == samplesize) {
//     unsigned long sampanalysis[analysize];
//     for (byte i = 0; i < analysize; i++) {
//       sampanalysis[i] = samples[i + 1];
//       if (sampanalysis[i] > maxim) maxim = sampanalysis[i];
//       if (sampanalysis[i] < minim) minim = sampanalysis[i];
//       averg += sampanalysis[i];
//       stdevi += sampanalysis[i] * sampanalysis[i];
//     }

//     averg = averg / analysize;
//     stdevi = sqrt(stdevi / analysize - (float)averg * (float)averg);
//     if (stdevi < 1) stdevi = 1.0;
//     delta = maxim - minim;

//     if (baseline == 0) {
//       baseline = delta;
//     }

//     const float TOUCH_OFFSET = 150.0;
//     const float REL_OFFSET = 80.0;

//     // ============================================
//     // TOUCH DETECTION
//     // ============================================
//     if (!isTouched) {
//       baseline = (baseline * 0.95) + ((float)delta * 0.05);
      
//       if (delta > (baseline + TOUCH_OFFSET)) {
//         isTouched = true;
//         touchStartTime = millis();
//         Serial.println(">>> TOUCH STARTED <<<");
//       }
//     } else {
//       if (millis() - touchStartTime > 10000) {
//         baseline = delta;
//         isTouched = false;
        
//         // Turn off LEDs
//         for (byte i = 0; i < LED_NUM - 1; i++) {
//           leds[i].stop_fade();
//           leds[i].set_value(0);
//         }
        
//         Serial.println(">>> AUTO-RELEASE <<<");
//       }
//       else if (delta < (baseline + REL_OFFSET)) {
//         isTouched = false;
        
//         // Turn off LEDs
//         for (byte i = 0; i < LED_NUM - 1; i++) {
//           leds[i].stop_fade();
//           leds[i].set_value(0);
//         }
        
//         Serial.println(">>> TOUCH RELEASED <<<");
//       }
//     }

//     // ============================================
//     // MUSIC GENERATION (With Proper Timing!)
//     // ============================================
//     byte change = 0;
    
//     if (isTouched) {
//         if (delta > 100) {
//             change = 1;
//         }
//     } else {
//         if (delta > (stdevi * threshold)) {
//             change = 1;
//         }
//     }

//     // *** KEY FIX: Only generate notes at reasonable intervals ***
//     static unsigned long lastNoteTime = 0;
    
//     if (change && (millis() - lastNoteTime > 200)) {  // Minimum 200ms between notes
//         lastNoteTime = millis();
        
//         // int duration = 150 + (map(delta % 127, 1, 127, 100, 1500));  // Shorter max duration
//         int duration = 800 + (map(delta % 127, 1, 127, 500, 2000));  // 800-2800ms
//         int ramp = 3 + (duration % 100);
//         int notechannel = random(1, 5);
        
//         int setnote = map(averg % 127, 1, 127, noteMin, noteMax);
//         setnote = scaleNote(setnote, scale[currScale], root);

//         int velocity;
//         if (isTouched) {
//             velocity = 127;
            
//             // LEDs when touched
//             if (noteLEDs > 0) {
//                 rampUp(random(0, LED_NUM - 1), 255, 30);
//             }
//         } else {
//             velocity = 60;
//         }

//         // Play note
//         if (QY8) {
//             setNote(setnote, velocity, duration, notechannel);
//         } else {
//             setNote(setnote, velocity, duration, channel);
//         }
        
//         setControl(controlNumber, controlMessage.value, delta % 127, ramp);
//     }

//     // ============================================
//     // DATA STREAMING (Every 50ms)
//     // ============================================
//     static unsigned long lastStream = 0;
//     if (millis() - lastStream > 50) {
//        Serial.print("D,");
//        Serial.println(delta);
//        lastStream = millis();
//     }

//     index = 0;
//   }
// }


// claude second
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
      if (sampanalysis[i] > maxim) maxim = sampanalysis[i];
      if (sampanalysis[i] < minim) minim = sampanalysis[i];
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];
    }

    averg = averg / analysize;
    stdevi = sqrt(stdevi / analysize - (float)averg * (float)averg);
    if (stdevi < 1) stdevi = 1.0;
    delta = maxim - minim;

    if (baseline == 0) {
      baseline = delta;
    }

    const float TOUCH_OFFSET = 150.0;
    const float REL_OFFSET = 80.0;

    // ============================================
    // TOUCH DETECTION
    // ============================================
    if (!isTouched) {
      baseline = (baseline * 0.95) + ((float)delta * 0.05);
      
      if (delta > (baseline + TOUCH_OFFSET)) {
        isTouched = true;
        touchStartTime = millis();
        Serial.println(">>> TOUCH STARTED <<<");
      }
    } else {
      if (millis() - touchStartTime > 10000) {
        baseline = delta;
        isTouched = false;
        
        for (byte i = 0; i < LED_NUM - 1; i++) {
          leds[i].stop_fade();
          leds[i].set_value(0);
        }
        
        Serial.println(">>> AUTO-RELEASE <<<");
      }
      else if (delta < (baseline + REL_OFFSET)) {
        isTouched = false;
        
        for (byte i = 0; i < LED_NUM - 1; i++) {
          leds[i].stop_fade();
          leds[i].set_value(0);
        }
        
        Serial.println(">>> TOUCH RELEASED <<<");
      }
    }

    // ============================================
    // MUSIC GENERATION (Like Original!)
    // ============================================
    byte change = 0;
    
    if (isTouched) {
        if (delta > 100) {
            change = 1;
        }
    } else {
        if (delta > (stdevi * threshold)) {
            change = 1;
        }
    }

    // *** REMOVED TIMING RESTRICTION - Just like original! ***
    if (change) {
        int duration = 150 + (map(delta % 127, 1, 127, 100, 2500));
        int ramp = 3 + (duration % 100);
        int notechannel = random(1, 5);
        
        int setnote = map(averg % 127, 1, 127, noteMin, noteMax);
        setnote = scaleNote(setnote, scale[currScale], root);

        int velocity;
        if (isTouched) {
            velocity = 127;
            
            if (noteLEDs > 0) {
                rampUp(random(0, LED_NUM - 1), 255, 30);
            }
        } else {
            velocity = 60;
        }

        // Play note - polyphony system handles overlaps!
        if (QY8) {
            setNote(setnote, velocity, duration, notechannel);
        } else {
            setNote(setnote, velocity, duration, channel);
        }
        
        setControl(controlNumber, controlMessage.value, delta % 127, ramp);
    }

    // ============================================
    // DATA STREAMING
    // ============================================
    static unsigned long lastStream = 0;
    if (millis() - lastStream > 50) {
       Serial.print("D,");
       Serial.println(delta);
       lastStream = millis();
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
    } // highest scale value less than or equal to note
    // otherwise continue search
  }
  // didn't find note and didn't pass note value, uh oh!
  return 6; // give arbitrary value rather than fail
}

int scaleNote(int note, int scale[], int root) {
  // input note mod 12 for scaling, note/12 octave
  // search array for nearest note, return scaled*octave
  int scaled = note % 12;
  int octave = note / 12;
  int scalesize = (scale[0]);
  // search entire array and return closest scaled note
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + root; // apply octave and root
  return scaled;
}
