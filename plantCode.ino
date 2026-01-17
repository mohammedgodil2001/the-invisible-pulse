

// Pin Definitions
const byte interruptPin = INT1;  // Galvanometer input (555 timer)
const byte knobPin = A0;         // Potentiometer analog input
const byte buttonPin = A1;       // Tactile button input

// LED Pin Array
int ledNums[6] = {3, 6, 10, 5, 9, 11};

// Timing Variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 1;

void setup() {
  // Initialize pin modes
  pinMode(knobPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Initialize serial communication for data output
  Serial.begin(115200);
  
  Serial.println("Plant Biodata Sensor Initialized");
  Serial.println("Waiting for sensor input...");
}

void loop() {
  // Update current time
  currentMillis = millis();
  
  // Main loop will be populated with sensor reading logic
  // Currently just maintains timing
}