#include <TM1637Display.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Define pins for TM1637 CLK and DIO
#define CLK PB11  // STM32 pin connected to TM1637 CLK
#define DIO PB10  // STM32 pin connected to TM1637 DIO

// Define pins for buttons
#define MODE_BUTTON_PIN PA2        // STM32 pin connected to mode button
#define BRIGHTNESS_BUTTON_PIN PA3  // STM32 pin connected to brightness button
#define POWER_BUTTON_PIN PA4       // STM32 pin connected to power button

// I2C pins for STM32 Blue Pill (Hardware I2C1)
// SDA = PB7 (fixed hardware pin)
// SCL = PB6 (fixed hardware pin)
// Connect ADS1115: VDD->3.3V, GND->GND, SDA->PB7, SCL->PB6

// ADS1115 ADC configuration (16-bit)
Adafruit_ADS1115 ads;  // Create ADS1115 instance

// Calibration data points from actual measurements
// millivolts => psi
// ~ 2260 => 50 psi
// ~ 1850 => 40 psi
// ~ 1555 => 30 psi
// ~ 1255 => 25 psi

// Using linear interpolation between calibration points
struct CalibrationPoint {
  int millivolts;
  float psi;
};

// Calibration lookup table (sorted by millivolts)
const CalibrationPoint calibrationTable[] = {
  {1255, 27.0},
  {1555, 32.0},
  {1855, 40.0},
  {2260, 51.0}
};
const int calibrationPoints = sizeof(calibrationTable) / sizeof(calibrationTable[0]);

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
int mode = 0;           // Mode: 0 = "Bar", 1 = "PSI", 2 = "raw"
int brightness = 0;     // Initial brightness level
bool displayOn = true;  // Display power state

// Button press flags (independent for each button)
bool modeButtonPressed = false;
bool brightnessButtonPressed = false;
bool powerButtonPressed = false;

// Timer for mode display
unsigned long modeDisplayTime = 0;

// Exponential Moving Average (EMA)
float filteredVoltage = 0;
const float alpha = 0.1;  // Increased alpha for faster response (was 0.1)

void setup() {
  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(100000);  // Back to conservative 100kHz
  
  if (!ads.begin(0x48)) {
    if (!ads.begin(0x49)) {
      display.setSegments(new uint8_t[4]{ E, r, r, empty });
      while(1) delay(1000);
    }
  }
  
  // Use GAIN_ONE for better range up to 4.096V (safer for 4.5V max)
  ads.setGain(GAIN_ONE);    // +/- 4.096V, 1 bit = 0.125mV (good for 0.5-4.5V range)
  ads.setDataRate(128);     // Faster rate for quicker response (was 16)
  
  delay(500);  // Longer stabilization
  
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);        // Initialize mode button pin with internal pull-up
  pinMode(BRIGHTNESS_BUTTON_PIN, INPUT_PULLUP);  // Initialize brightness button pin with internal pull-up
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);       // Initialize power button pin with internal pull-up

  resetValue();
  display.setBrightness(brightness);  // Set initial brightness level
  updateDisplayMode();
  modeDisplayTime = millis();  // Start timer
}

void loop() {
  // Check mode button press
  if (checkButtonPress(MODE_BUTTON_PIN)) {
    mode = (mode + 1) % 3;  // Cycle through modes
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

  // Switch to selected function after 1.5 seconds
  if (millis() - modeDisplayTime > 1500 && displayOn) {
    switch (mode) {
      case 0:
        pressureMode(false);  // Bar mode
        break;
      case 1:
        pressureMode(true);   // PSI mode
        break;
      case 2:
        rawValueMode();       // Raw voltage mode
        break;
    }
  }

  delay(50);  // Faster loop for quicker response (was 50ms)
}

void rawValueMode() {
  // Single reading for faster response
  int16_t adcValue = ads.readADC_SingleEnded(0);
  float voltage = ads.computeVolts(adcValue);
  
  int millivolts = (int)(voltage * 1000);
  
  // Display with bounds checking
  if (millivolts < 0) millivolts = 0;
  if (millivolts > 9999) millivolts = 9999;
  
  display.showNumberDec(millivolts);
}

// Function to convert millivolts to PSI using calibration table
float millvoltsToPSI(int millivolts) {
  // Handle values below minimum calibration point
  if (millivolts <= calibrationTable[0].millivolts) {
    // Extrapolate below minimum point or return 0
    if (millivolts < 1000) return 0.0;  // Below reasonable sensor range
    
    // Linear extrapolation from first two points
    float slope = (calibrationTable[1].psi - calibrationTable[0].psi) / 
                  (calibrationTable[1].millivolts - calibrationTable[0].millivolts);
    return calibrationTable[0].psi + slope * (millivolts - calibrationTable[0].millivolts);
  }
  
  // Handle values above maximum calibration point
  if (millivolts >= calibrationTable[calibrationPoints-1].millivolts) {
    // Extrapolate above maximum point
    int lastIdx = calibrationPoints - 1;
    float slope = (calibrationTable[lastIdx].psi - calibrationTable[lastIdx-1].psi) / 
                  (calibrationTable[lastIdx].millivolts - calibrationTable[lastIdx-1].millivolts);
    float extrapolated = calibrationTable[lastIdx].psi + slope * (millivolts - calibrationTable[lastIdx].millivolts);
    
    // Cap at reasonable maximum (e.g., 100 PSI)
    return (extrapolated > 100.0) ? 100.0 : extrapolated;
  }
  
  // Linear interpolation between calibration points
  for (int i = 0; i < calibrationPoints - 1; i++) {
    if (millivolts >= calibrationTable[i].millivolts && millivolts <= calibrationTable[i+1].millivolts) {
      float slope = (calibrationTable[i+1].psi - calibrationTable[i].psi) / 
                    (calibrationTable[i+1].millivolts - calibrationTable[i].millivolts);
      return calibrationTable[i].psi + slope * (millivolts - calibrationTable[i].millivolts);
    }
  }
  
  // Fallback (should not reach here)
  return 0.0;
}

void pressureMode(bool isPsi) {
  // Read ADC value and convert to voltage
  int16_t adcValue = ads.readADC_SingleEnded(0);
  float voltage = ads.computeVolts(adcValue);

  // EMA filtering
  filteredVoltage = alpha * voltage + (1 - alpha) * filteredVoltage;
  
  // Convert filtered voltage to millivolts for calibration lookup
  int millivolts = (int)(filteredVoltage * 1000);
  
  // Use calibration table to get accurate PSI reading
  float pressurePSI = millvoltsToPSI(millivolts);
  
  // Ensure pressure is within reasonable range
  if (pressurePSI < 0) pressurePSI = 0;
  if (pressurePSI > 100) pressurePSI = 100;

  // Display result
  int scaledValue;
  if (isPsi) {
    scaledValue = (int)(pressurePSI * 10);
    display.showNumberDecEx(scaledValue, 0b00100000);  // "xxx.x"
  } else {
    float pressureBar = pressurePSI * PSI_TO_BAR;
    scaledValue = (int)(pressureBar * 100);
    display.showNumberDecEx(scaledValue, 0b01000000);  // "xx.xx"
  }
}

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
    }
  }
}

void resetValue() {
  mode = 0;
  brightness = 0;
  displayOn = true;
  filteredVoltage = 0;  // Reset filtered voltage
}
