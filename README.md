//Component,Wire Color,Connects to ESP32 Pin,Purpose
//Servo Motor,Red,5V (VIN/VBUS),"CRITICAL POWER: Must be 5V, not 3.3V."
//Servo Motor,Yellow/Orange,GPIO 2,Signal pin for PWM control.
//Potentiometer,Center Pin,GPIO 34,Analog input pin for power control.
//Potentiometer,Outer Pins,3.3V and GND,Power for the sensor.




#include <WiFi.h>
#include <ThingSpeak.h>
#include <Keypad.h>
#include <ESP32Servo.h>

// =========================================================
// 1. CONFIGURATION
// =========================================================
char ssid[] = "";                  // Your Wi-Fi Network Name
char pass[] = "";               // Your Wi-Fi Password

unsigned long myChannelNumber = 123456;         // Your ThingSpeak Channel ID
const char * myWriteAPIKey = "lalapoopoo"; // Your ThingSpeak Write API Key

// Keypad Configuration (4x3)
const byte ROWS = 4; 
const byte COLS = 3; 
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {15, 13, 12, 14}; // Connect Keypad Row wires here
byte colPins[COLS] = {27, 26, 25};    // Connect Keypad Column wires here

// Secret PIN and input buffer
const String SECRET_PIN = "1234#"; 
const int PIN_LENGTH = 5; 

// Pin Definitions
#define POT_PIN 34     // ESP32 Analog Pin for Potentiometer 
#define SERVO_PIN 2    // ESP32 PWM Pin for the Servo Motor 

// Servo Pulse Width Definitions (For reliable 0-180 movement)
#define MIN_PW 500   // Min pulse width (0 degrees)
#define MAX_PW 2400  // Max pulse width (180 degrees)

// =========================================================
// 2. GLOBAL OBJECTS AND VARIABLES
// =========================================================
WiFiClient client;
Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Servo myServo;

String pinBuffer = "";
int currentLaunchAngle = 0; // Stores the calculated angle for logging

// =========================================================
// 3. FUNCTION DECLARATIONS
// =========================================================
void logLaunchData(int power, int angle); // Forward declaration

// =========================================================
// 4. SETUP FUNCTION (Runs Once)
// =========================================================
void setup() {
  Serial.begin(115200);

  // --- Hardware Initialization ---
  pinMode(POT_PIN, INPUT);           
  
  // Initialize Servo Motor
  ESP32PWM::allocateTimer(0);        
  
  // Attach with defined pulse widths for reliable 0-180 movement
  myServo.attach(SERVO_PIN, MIN_PW, MAX_PW); 
  
  myServo.write(0);                  // Set initial locked position (0 degrees)

  // --- Wi-Fi Connection ---
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. ThingSpeak ready.");
    ThingSpeak.begin(client);
  } else {
    Serial.println("\nWiFi FAILED to connect. Logging will be skipped.");
  }
  
  Serial.println("System READY. Enter PIN (1234#) on the Keypad to launch.");
}

// =========================================================
// 5. LOOP FUNCTION (Runs Forever)
// =========================================================
void loop() {
  
  // A. Read Potentiometer Value Constantly 
  int potValue = analogRead(POT_PIN);
  
  // Map 1: For Logging (0-100 power percentage)
  int launchPowerForLog = map(potValue, 0, 4095, 0, 100); 

  // B. Check for Key Presses
  char customKey = customKeypad.getKey();

  if (customKey != NO_KEY) {
    pinBuffer += customKey;
    Serial.print(customKey); 
    
    // C. Check Pin Buffer Length
    if (pinBuffer.length() >= PIN_LENGTH) {
      Serial.println(); 

      if (pinBuffer == SECRET_PIN) {
        // --- ACCESS GRANTED ---
        // logic: Potentiometer controls Servo ONLY HERE
        
        // 1. DIRECT CONTROL (Maps raw Pot to Pulse Width 500us - 2400us)
        int releasePulse = map(potValue, 0, 4095, MIN_PW, MAX_PW); 
        
        // 2. LOGGING ANGLE (Maps raw Pot to a clean 0-180 degree number)
        currentLaunchAngle = map(potValue, 0, 4095, 0, 180);

        Serial.print("ACCESS GRANTED. Angle: ");
        Serial.print(currentLaunchAngle);
        Serial.println(" degrees.");

        // 3. **PERFORM LAUNCH ACTION (Servo moves)**
        myServo.writeMicroseconds(releasePulse); 
        delay(500);        
        myServo.write(0);  // Return to locked position (0 degrees)

        // 4. **LOG DATA**
        if (WiFi.status() == WL_CONNECTED) {
            logLaunchData(launchPowerForLog, currentLaunchAngle); 
        } 

      } else {
        // --- ACCESS DENIED ---
        Serial.println("ACCESS DENIED. Incorrect PIN.");
      }
      
      // D. Reset Buffer
      pinBuffer = ""; 
    }
  }
  delay(10); 
}

// =========================================================
// 6. LOGGING FUNCTION
// =========================================================
void logLaunchData(int power, int angle) {
  
  // Set the data fields for ThingSpeak
  ThingSpeak.setField(1, power);    // Field 1: Launch Power (0-100)
  ThingSpeak.setField(3, 1);        // Field 3: Launch Status (1 for success)
  ThingSpeak.setField(5, angle);    // Field 5: Launch Angle (0-180)

  // Send the data
  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (httpCode == 200) {
    Serial.println("Launch data successfully logged to ThingSpeak.");
  } else {
    Serial.print("ThingSpeak write failed. Error code: ");
    Serial.println(httpCode);
  }
}
