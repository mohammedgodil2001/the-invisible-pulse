// ===================================
// PLANT BIODATA SENSOR
// Interactive Projection Mapping
// ===================================

// Pin Definitions
const byte interruptPin = INT1;  // Galvanometer input (555 timer)
const byte knobPin = A0;         // Potentiometer analog input
const byte buttonPin = A1;       // Tactile button input

// LED Pin Array
int ledNums[6] = {3, 6, 10, 5, 9, 11};

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
  Serial.println("Waiting for sensor input...");
  
  attachInterrupt(interruptPin, sample, RISING);
  Serial.println("Interrupt attached - ready to capture pulses");
}

void loop() {
  currentMillis = millis();
  
  if (index >= samplesize) {
    analyzeSample();
  }
}

void sample() {
  if (index < samplesize) {
    samples[index] = micros() - microseconds;
    microseconds = samples[index] + microseconds;
    index += 1;
  }
}

void analyzeSample() {
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;

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

    // Output sensor data
    Serial.print("Delta: ");
    Serial.print(delta);
    Serial.print(" | Average: ");
    Serial.print(averg);
    Serial.print(" | StdDev: ");
    Serial.println(stdevi);

    index = 0;
  }
}