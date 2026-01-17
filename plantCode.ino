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

void setup() {
  // Initialize pin modes
  pinMode(knobPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Initialize serial communication for data output
  Serial.begin(115200);
  
  Serial.println("Plant Biodata Sensor Initialized");
  Serial.println("Waiting for sensor input...");
  
  // Attach interrupt to capture 555 timer pulses
  // RISING = trigger on rising edge of pulse
  attachInterrupt(interruptPin, sample, RISING);
  Serial.println("Interrupt attached - ready to capture pulses");
}

void loop() {
  // Update current time
  currentMillis = millis();
  
  // Check if sample array is full
  if (index >= samplesize) {
    // For now, just reset and print status
    Serial.print("Captured ");
    Serial.print(samplesize);
    Serial.println(" samples");
    
    // Print first few samples for debugging
    Serial.print("Sample values: ");
    for (byte i = 0; i < 3; i++) {
      Serial.print(samples[i]);
      Serial.print(" ");
    }
    Serial.println("...");
    
    // Reset index to capture new samples
    index = 0;
  }
}

// Interrupt Service Routine (ISR)
// This function is called automatically every time the 555 timer pulses
void sample() {
  // Only collect samples if array isn't full
  if (index < samplesize) {
    // Calculate time since last pulse
    samples[index] = micros() - microseconds;
    
    // Update microseconds counter for next pulse
    microseconds = samples[index] + microseconds;
    
    // Move to next array position
    index += 1;
  }
}