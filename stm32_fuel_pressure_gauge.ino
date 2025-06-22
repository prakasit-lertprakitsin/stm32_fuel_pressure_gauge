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
#define MULTIPLY_MAX 1.5000   // Maximum multiply value

/*
rawValue => real psi
~ 2047 => 50 psi
~ 1628 => 44 psi
~ 1194 => 34 psi
~ 826 => 28 psi
~ 688 => 25 psi
*/
#define PRESSURE_SLOPE 0.01855
#define PRESSURE_OFFSET 13.9

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
  analogReadResolution(12);
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
  // int rawResistorValue = analogRead(VAR_RESISTOR_PIN);
  // display.showNumberDec(rawResistorValue);

  float multiply = readMultiplyValue();
  float percentage = (multiply - 1.0) * 100.0;       // Convert to percentage
  int scaledValue = (int)(percentage * 100);         // Scale by 100 for 2 decimal places
  display.showNumberDecEx(scaledValue, 0b01000000);  // Display as "x.xxx"
}

void pressureMode(bool isPsi) {
  int rawValue = analogRead(SENSOR_PIN);

  // EMA filtering
  filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;

  // แปลง raw ADC → pressure โดยตรง
  float pressurePSI = filteredValue * PRESSURE_SLOPE + PRESSURE_OFFSET;

  // Apply multiply factor (เช่นใช้ปรับความแม่นหรือเทียบกับ sensor อื่น)
  float multiply = readMultiplyValue();
  pressurePSI *= multiply;

  // แสดงผล
  int scaledValue;
  if (isPsi) {
    scaledValue = (int)(pressurePSI * 10);  // xxx.x
    display.showNumberDecEx(scaledValue, 0b00100000);
  } else {
    float pressureBar = pressurePSI * PSI_TO_BAR;
    scaledValue = (int)(pressureBar * 100);  // xx.xx
    display.showNumberDecEx(scaledValue, 0b01000000);
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
  int constrainedValue = constrain(rawResistorValue, 36, 4065);

  // Map ADC value (36-4065) to multiply range (1.0000-1.2000)
  float targetMultiply = MULTIPLY_MIN + ((float)(constrainedValue - 36) / (float)(4065.0 - 36.0)) * (MULTIPLY_MAX - MULTIPLY_MIN);

  if (targetMultiply < 1.008) {
    return MULTIPLY_MIN;
  }

  return targetMultiply;
}

void resetValue() {
  mode = 0;
  brightness = 0;
  displayOn = true;
}
