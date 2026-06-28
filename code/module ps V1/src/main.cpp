#include "crgb.h"
#include <Arduino.h>
#include <ModbusMaster.h>
#include <ESP32Encoder.h>
#include <FastLED.h>

// ==========================================
// DEBUG MACROS
// ==========================================
#define DEBUG_SERIAL       // Comment out this line to disable all debug output

#ifdef DEBUG_SERIAL
  #define DBG(x)       Serial.println(x)
  #define DBGP(x)      Serial.print(x)
  #define DBGF(...)    Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGP(x)
  #define DBGF(...)
#endif

// ==========================================
// PIN DEFINITIONS (ESP32-S3)
// ==========================================
// Modbus UART Pins
#define MODBUS_RX_PIN 16
#define MODBUS_TX_PIN 17

// Voltage Encoder Pins
#define ENCODER_V_CLK 4
#define ENCODER_V_DT  5

// Current Encoder Pins
#define ENCODER_I_CLK 6
#define ENCODER_I_DT  7

// Output enable button
#define BUTTON_PIN 8

// Led info
#define LED_DATA_PIN 9
#define LED_ARRAY_LENGTH 5
#define LED_COLOUR_ORDER GRB
#define LED_TYPE WS2812B
#define LED_BRIGHTNESS 100

// ==========================================
// GLOBALS & OBJECTS
// ==========================================
ModbusMaster node;
ESP32Encoder encoderVoltage;
ESP32Encoder encoderCurrent;
CRGB leds[LED_ARRAY_LENGTH];

// XY6020L Modbus Multipliers (usually x100)
const float VOLTAGE_MULTIPLIER = 100.0;
const float CURRENT_MULTIPLIER = 100.0;

// Maximum and Minimum limits (Adjust to your needs)
const float MAX_VOLTAGE = 60.0;
const float MAX_CURRENT = 20.0;

// State tracking
float targetVoltage = 0.0;
float targetCurrent = 0.0;

int64_t lastEncoderVPos = 0;
int64_t lastEncoderIPos = 0;

// Timer for non-blocking read
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000; // Read actual output every 1000ms


// State tracking for the output and button debounce
bool outputActive = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce time

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void setTargetVoltage(float voltage);
void setTargetCurrent(float current);
void readActualOutput();
void setOutputState(bool state);
void check_voltage_encoder();
void check_current_encoder();
void activate_power();
void reset_power();

// ==========================================
// SETUP
// ==========================================
void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(115200);
    delay(100); // Give serial monitor time to connect
  #endif

  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,LED_COLOUR_ORDER>(leds, LED_ARRAY_LENGTH);
  FastLED.setBrightness(LED_BRIGHTNESS);

  for(int i = 0; i <= LED_ARRAY_LENGTH; i++) {
      leds[1] = CRGB(0,0,0); // not started jet
    }
  FastLED.show();

  DBG("Initializing Lab Bench Power Supply...");

  // --- 1. Init Modbus Communication ---
  Serial1.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  node.begin(1, Serial1); // 1 is default Slave ID for XY6020L
  DBG("Modbus UART Initialized.");

  // --- 2. Init Encoders ---
  // Most rotary encoders connect to GND, so internal PULLUP is recommended
  // GEWIJZIGD: UP veranderd naar enc_pull_t::UP
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoderVoltage.attachHalfQuad(ENCODER_V_DT, ENCODER_V_CLK);
  encoderCurrent.attachHalfQuad(ENCODER_I_DT, ENCODER_I_CLK);

  encoderVoltage.setCount(0);
  encoderCurrent.setCount(0);

  // Configure the button pin with an internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Ensure the power supply output is safely turned OFF at startup
  setOutputState(outputActive);
  
  DBG("Hardware Encoders Initialized.");

  // --- 3. Set Initial Safe Targets (0V / 0A) ---
  setTargetVoltage(targetVoltage);
  setTargetCurrent(targetCurrent);

  delay(500);

  for(int i = 0; i <= LED_ARRAY_LENGTH; i++) {
      leds[1] = CRGB(0,255,0); // ready for use
    }
  FastLED.show();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  
  // 3. Read actual output periodically (Non-blocking)
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();
    readActualOutput();
  }

  activate_power();

  // Small delay for loop stability
  delay(5);
}

// ==========================================
// MODBUS FUNCTIONS
// ==========================================

/**
 * Writes the target voltage to Modbus Register 0x0000
 */
void setTargetVoltage(float voltage) {
  uint16_t vSet = (uint16_t)(voltage * VOLTAGE_MULTIPLIER);
  uint8_t result = node.writeSingleRegister(0x0000, vSet);
  
  if (result != node.ku8MBSuccess) {
    DBGF("Modbus Error: Failed to set Voltage. Code: %d\n", result);
  }
}

/**
 * Writes the target current limit to Modbus Register 0x0001
 */
void setTargetCurrent(float current) {
  uint16_t iSet = (uint16_t)(current * CURRENT_MULTIPLIER);
  uint8_t result = node.writeSingleRegister(0x0001, iSet);
  
  if (result != node.ku8MBSuccess) {
    DBGF("Modbus Error: Failed to set Current. Code: %d\n", result);
  }
}

/**
 * Reads actual output Voltage (0x0002) and Current (0x0003)
 */
void readActualOutput() {
  // Request 2 registers starting at 0x0002
  uint8_t result = node.readHoldingRegisters(0x0002, 2);

  if (result == node.ku8MBSuccess) {
    uint16_t rawVoltage = node.getResponseBuffer(0);
    uint16_t rawCurrent = node.getResponseBuffer(1);

    float actualVoltage = (float)rawVoltage / VOLTAGE_MULTIPLIER;
    float actualCurrent = (float)rawCurrent / CURRENT_MULTIPLIER;

    DBGF("Output -> V: %.2f V | I: %.2f A\n", actualVoltage, actualCurrent);
  } else {
    DBGF("Modbus Error: Failed to read actual values. Code: %d\n", result);
  }
}

/**
  Sends a Modbus command to turn the physical output ON or OFF.
  @param state true to enable output, false to disable output
 */
void setOutputState(bool state) {
  uint16_t command = state ? 1 : 0;
  
  // Write to Register 0x0012 (Output Switch Register)
  uint8_t result = node.writeSingleRegister(0x0012, command);
  
  if (result == node.ku8MBSuccess) {
    DBGF("Modbus: Output successfully turned %s\n", state ? "ON" : "OFF");
  } else {
    DBGF("Modbus Error: Failed to change output state. Code: %d\n", result);
  }
}

void check_voltage_encoder() {
  // 1. Check Voltage Encoder
  int64_t currentVPos = encoderVoltage.getCount();
  if (currentVPos != lastEncoderVPos) {
    int direction = (currentVPos > lastEncoderVPos) ? 1 : -1;
    targetVoltage += (direction * 0.1); // 0.1V steps
    
    // Constrain values
    if (targetVoltage < 0.0) targetVoltage = 0.0;
    if (targetVoltage > MAX_VOLTAGE) targetVoltage = MAX_VOLTAGE;

    DBGF("New Voltage Target: %.2f V\n", targetVoltage);
    
    // Update power supply
    setTargetVoltage(targetVoltage);
    
    lastEncoderVPos = currentVPos;
  }
}

void check_current_encoder() {
  // 2. Check Current Encoder
  int64_t currentIPos = encoderCurrent.getCount();
  if (currentIPos != lastEncoderIPos) {
    int direction = (currentIPos > lastEncoderIPos) ? 1 : -1;
    targetCurrent += (direction * 0.1); // 0.1A steps
    
    // Constrain values
    if (targetCurrent < 0.0) targetCurrent = 0.0;
    if (targetCurrent > MAX_CURRENT) targetCurrent = MAX_CURRENT;

    DBGF("New Current Target: %.2f A\n", targetCurrent);
    
    // Update power supply
    setTargetCurrent(targetCurrent);
    
    lastEncoderIPos = currentIPos;
  }
}


void activate_power() {
  // Read the current state of the button
  bool reading = digitalRead(BUTTON_PIN);
  
  // Check if the button state has changed (due to noise or pressing)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If the state has persisted longer than the debounce delay, it's a valid press
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Check if the button was pushed down (transitions from HIGH to LOW)
    if (reading == LOW && outputActive == false && lastButtonState == HIGH) {
      // Toggle state to ON
      outputActive = true;
      
      for(int i = 0; i <= LED_ARRAY_LENGTH; i++) {
        leds[1] = CRGB(255,0,0); // output turned on
      }
      FastLED.show();
      
      setOutputState(outputActive);
    } 
    else if (reading == LOW && outputActive == true && lastButtonState == HIGH) {
      // Toggle state to OFF
      outputActive = false;
      
      for(int i = 0; i <= LED_ARRAY_LENGTH; i++) {
        leds[1] = CRGB(0,0,255); // wayting for output at 0
      }
      FastLED.show();
      
      setOutputState(outputActive);
    }
  }
  
  // Save the reading for the next iteration
  lastButtonState = reading;
}

void reset_power() {
  setTargetVoltage(targetVoltage);
  setTargetCurrent(targetCurrent);
}