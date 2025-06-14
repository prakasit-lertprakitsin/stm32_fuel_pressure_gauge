#include <TM1637Display.h>

// Define pins for TM1637 CLK and DIO
#define CLK PB11  // STM32 pin connected to TM1637 CLK
#define DIO PB10  // STM32 pin connected to TM1637 DIO

// Define pins for buttons
#define MODE_BUTTON_PIN PA2        // STM32 pin connected to mode button
#define BRIGHTNESS_BUTTON_PIN PA3  // STM32 pin connected to brightness button
#define POWER_BUTTON_PIN PA4       // STM32 pin connected to power button

// Pressure sensor configuration
#define SENSOR_PIN PA0  // Analog input pin for pressure sensor

// Variable resistor configuration
#define VAR_RESISTOR_PIN PA1  // Analog input pin for variable resistor
#define MULTIPLY_MIN 1.0000   // Minimum multiply value
#define MULTIPLY_MAX 1.2000   // Maximum multiply value

// Calibration constants for pressure transducer
// #define VOLTAGE_OFFSET 0.5  // Voltage at 0 PSI (adjust based on your sensor)
// #define VOLTAGE_SCALE 0.04  // Voltage per PSI (adjust based on your sensor)

#define PRESSURE_SLOPE 0.08197  // PSI per raw ADC unit
#define PRESSURE_OFFSET 15.901  // PSI offset

unsigned long zeroStartTime = 0;                                  // Time when rawValue first became 0
bool isInZeroMode = false;                                        // Flag to track if we're in zero mock mode
const unsigned long ZERO_TRANSITION_TIME = 30UL * 60UL * 1000UL;  // 30 minutes in milliseconds
// const unsigned long ZERO_TRANSITION_TIME = 5UL * 60UL * 1000UL;   // 5 minutes in milliseconds
const float INITIAL_ZERO_PRESSURE = 15.901;  // Starting pressure when rawValue = 0

bool isFirstStart = true;

// Conversion factor: 1 PSI = 0.0689476 bar
#define PSI_TO_BAR 0.0689476

// Create an instance of the TM1637Display class
TM1637Display display(CLK, DIO);

// Define custom characters for "P", "E", "S", "B", "W", "T", "M"
const uint8_t P = 0b01110011;  // P: Segments a, b, e, f, g
const uint8_t E = 0b01111001;  // E: Segments a, d, e, f, g
const uint8_t S = 0b01101101;
const uint8_t B = 0b01111111;      // B: Segments a, b, c, d, e, f
const uint8_t W = 0b00101010;      // W: Segments b, c, e
const uint8_t T = 0b01111000;      // T: Segments a, e, f
const uint8_t M = 0b00010101;      // M: Segments a, c, e
const uint8_t A = 0b01110111;      // A: Segments a, b, c, e, f
const uint8_t R = 0b01110001;      // R: Segments a, b, e, f
const uint8_t U = 0b00111110;      // U: Segments b, c, d, e, f
const uint8_t L = 0b00111000;      // L: Segments d, e, f
const uint8_t I = 0b00110000;      // I: Segments b, c
const uint8_t r = 0b01010000;      // r: Segments a, e
const uint8_t a = 0b01010100;      // a: Segments a, c, e
const uint8_t empty = 0b00000000;  // Empty: No segments

// Variables to track state
int mode = 0;           // Mode: 0 = "BPES", 1 = "PPES", 2 = "WTEM"
int brightness = 0;     // Initial brightness level
bool displayOn = true;  // Display power state

// Button press flags (independent for each button)
bool modeButtonPressed = false;
bool brightnessButtonPressed = false;
bool powerButtonPressed = false;

// Timer for mode display
unsigned long modeDisplayTime = 0;

// Exponential Moving Average (EMA)
float filteredValue = 0;
const float alpha = 0.1;

void setup() {
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);        // Initialize mode button pin with internal pull-up
  pinMode(BRIGHTNESS_BUTTON_PIN, INPUT_PULLUP);  // Initialize brightness button pin with internal pull-up
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);       // Initialize power button pin with internal pull-up

  pinMode(SENSOR_PIN, INPUT_ANALOG);
  pinMode(VAR_RESISTOR_PIN, INPUT_ANALOG);  // Initialize variable resistor pin

  resetValue();
  display.setBrightness(brightness);  // Set initial brightness level
  updateDisplayMode();
  modeDisplayTime = millis();  // Start timer
}

void loop() {
  // Check mode button press
  if (checkButtonPress(MODE_BUTTON_PIN)) {
    mode = (mode + 1) % 4;  // Cycle through modes
    updateDisplayMode();
    modeDisplayTime = millis();  // Reset timer
  }

  // Check brightness button press
  if (checkButtonPress(BRIGHTNESS_BUTTON_PIN)) {
    brightness = (brightness + 1) % 8;  // Cycle through brightness levels (wrap from 7 to 0)
    display.setBrightness(brightness);
    updateDisplayMode();
    modeDisplayTime = millis();  // Reset timer
  }

  // Check power button press
  if (checkButtonPress(POWER_BUTTON_PIN)) {
    displayOn = !displayOn;  // Toggle display power state
    if (displayOn) {
      updateDisplayMode();
      modeDisplayTime = millis();  // Reset timer
    } else {
      display.clear();
    }
  }

  // Switch to selected function after 1 second
  if (millis() - modeDisplayTime > 1500 && displayOn) {
    switch (mode) {
      case 0:
        pressureMode(false);
        break;
      case 1:
        pressureMode(true);
        break;
      case 2:
        rawValueMode();
        break;
      case 3:
        multiplyMode();
        break;
    }
  }

  delay(50);  // Debounce delay for stability
}


void rawValueMode() {
  int rawValue = analogRead(SENSOR_PIN);
  display.showNumberDec(rawValue);
}

void multiplyMode() {
  float multiply = readMultiplyValue();
  float percentage = (multiply - 1.0) * 100.0;       // Convert to percentage
  int scaledValue = (int)(percentage * 100);         // Scale by 100 for 2 decimal places
  display.showNumberDecEx(scaledValue, 0b01000000);  // Display as "x.xxx"
}

void pressureMode(bool isPsi) {
  int rawValue = analogRead(SENSOR_PIN);

  // Exponential Moving Average (EMA)
  filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;

  float multiply = readMultiplyValue();
  float pressurePSI;

  if (isFirstStart) {
    if (rawValue > 2) {
      isFirstStart = false;
    } else {
      filteredValue = 0;
    }

    // Handle special case when rawValue = 0
  } else if (rawValue < 2 && !isFirstStart) {
    if (!isInZeroMode) {
      // First time entering zero mode
      isInZeroMode = true;
      zeroStartTime = millis();
      pressurePSI = INITIAL_ZERO_PRESSURE * multiply;
    } else {
      // We're in zero mode - calculate gradual transition
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - zeroStartTime;

      if (elapsedTime >= ZERO_TRANSITION_TIME) {
        // 30 minutes have passed, show 0 PSI
        pressurePSI = 0.0;
      } else {
        // Calculate gradual decrease from 15.901 to 0 over 30 minutes
        float transitionProgress = (float)elapsedTime / (float)ZERO_TRANSITION_TIME;
        float currentBasePressure = INITIAL_ZERO_PRESSURE * (1.0 - transitionProgress);
        pressurePSI = currentBasePressure * multiply;
      }
    }
  } else {
    // Normal operation - reset zero mode if we were in it
    if (isInZeroMode) {
      isInZeroMode = false;
      zeroStartTime = 0;
    }

    // Use smooth extrapolation for values below your calibrated range
    if (filteredValue < 172) {
      // Extrapolate linearly backwards from your calibration
      pressurePSI = ((filteredValue * PRESSURE_SLOPE) + PRESSURE_OFFSET) * multiply;

      // Optional: Prevent negative values if desired
      if (pressurePSI < 0) {
        pressurePSI = 0;
      }
    } else {
      // Use normal calibration for values >= 172
      pressurePSI = ((filteredValue * PRESSURE_SLOPE) + PRESSURE_OFFSET) * multiply;
    }
  }

  // Variable to hold the scaled value for display
  int scaledValue;
  if (isPsi) {
    // Scale PSI value by 10 to show one decimal place (e.g., "123.4")
    scaledValue = (int)(pressurePSI * 10);
    display.showNumberDecEx(scaledValue, 0b00100000);  // Display as "xxx.x"
  } else {
    // Convert PSI to bar and scale by 100 to show two decimal places (e.g., "12.34")
    float pressureBar = pressurePSI * PSI_TO_BAR;
    scaledValue = (int)(pressureBar * 100);
    display.showNumberDecEx(scaledValue, 0b01000000);  // Display as "xx.xx"
  }
}

// void pressureMode(bool isPsi) {
//   int rawValue = analogRead(SENSOR_PIN);

//   // Exponential Moving Average (EMA)
//   filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;

//   float multiply = readMultiplyValue();
//   float pressurePSI = ((filteredValue * PRESSURE_SLOPE) + PRESSURE_OFFSET) * multiply;

//   // Variable to hold the scaled value for display
//   int scaledValue;
//   if (isPsi) {
//     // Scale PSI value by 10 to show one decimal place (e.g., "123.4")
//     scaledValue = (int)(pressurePSI * 10);
//     display.showNumberDecEx(scaledValue, 0b00100000);  // Display as "xxx.x"
//   } else {
//     // Convert PSI to bar and scale by 100 to show two decimal places (e.g., "12.34")
//     float pressureBar = pressurePSI * PSI_TO_BAR;
//     scaledValue = (int)(pressureBar * 100);
//     display.showNumberDecEx(scaledValue, 0b01000000);  // Display as "xx.xx"
//   }
// }

// Function to check for button press (independent tracking for each button)
bool checkButtonPress(int buttonPin) {
  static unsigned long lastPressTime[3] = { 0 };  // Last press time for each button
  static bool previousState[3] = { HIGH };        // Previous state for each button

  int index;

  if (buttonPin == MODE_BUTTON_PIN) index = 0;
  else if (buttonPin == BRIGHTNESS_BUTTON_PIN) index = 1;
  else if (buttonPin == POWER_BUTTON_PIN) index = 2;

  bool currentState = digitalRead(buttonPin);  // Current state of the button

  if (previousState[index] == HIGH && currentState == LOW) {  // Button pressed event detected
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime[index] > 200) {  // Debounce logic (200ms)
      lastPressTime[index] = currentTime;
      previousState[index] = currentState;
      return true;  // Button press registered
    }
  } else if (previousState[index] == LOW && currentState == HIGH) {  // Button released event detected
    previousState[index] = currentState;
  }

  return false;  // No valid press detected
}

// Function to update display mode based on the current mode value
void updateDisplayMode() {
  if (displayOn) {
    switch (mode) {
      case 0:
        display.setSegments(new uint8_t[4]{ empty, B, a, r });  // Display "Bar"
        break;
      case 1:
        display.setSegments(new uint8_t[4]{ empty, P, S, I });  // Display "PSI"
        break;
      case 2:
        display.setSegments(new uint8_t[4]{ empty, r, a, W });  // Display "raw"
        break;
      case 3:
        display.setSegments(new uint8_t[4]{ empty, M, U, L });  // Display "MUL"
        break;
    }
  }
}

float readMultiplyValue() {
  int rawResistorValue = analogRead(VAR_RESISTOR_PIN);

  // Constrain the raw value to your actual range
  int constrainedValue = constrain(rawResistorValue, 8, 1016);

  // Map ADC value (8-1016) to multiply range (1.0000-1.2000)
  float targetMultiply = MULTIPLY_MIN + ((float)(constrainedValue - 8) / (float)(1016 - 8)) * (MULTIPLY_MAX - MULTIPLY_MIN);

  return targetMultiply;
}

void resetValue() {
  mode = 0;
  brightness = 0;
  displayOn = true;
}
