#include <TM1637Display.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Define pins for TM1637 CLK and DIO
#define CLK PB11  // STM32 pin connected to TM1637 CLK
#define DIO PB10  // STM32 pin connected to TM1637 DIO

// Define pin for single button
#define BUTTON_PIN PB12  // STM32 pin connected to the single button

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

const CalibrationPoint calibrationTable[] = {
  // milli voltage => real psi
  // Extended range 0-100 PSI

  // Low pressure range (0-15 PSI) - extrapolated from existing data
  { 50, 8.0 },    // Extrapolated
  { 75, 10.0 },   // Extrapolated
  { 95, 11.0 },   // Extrapolated
  { 115, 12.0 },  // Original
  { 135, 13.0 },  // Original
  { 155, 14.0 },  // Original
  { 175, 15.0 },  // Interpolated

  // Low-medium range (15-25 PSI) - interpolated
  { 235, 17.0 },  // Interpolated
  { 295, 19.0 },  // Interpolated
  { 355, 21.0 },  // Interpolated
  { 415, 23.0 },  // Interpolated
  { 505, 24.0 },  // Interpolated
  { 595, 25.0 },  // Original

  // Medium range (25-30 PSI) - based on existing data
  { 635, 26.0 },  // Original
  { 670, 27.0 },  // Original
  { 705, 28.0 },  // Interpolated
  { 777, 29.0 },  // Interpolated
  { 850, 30.0 },  // Original

  // Medium-high range (30-50 PSI) - based on existing data
  { 920, 32.0 },   // Interpolated
  { 978, 34.0 },   // Interpolated
  { 1035, 35.0 },  // Original
  { 1060, 37.0 },  // Interpolated
  { 1080, 38.0 },  // Original
  { 1148, 39.0 },  // Interpolated
  { 1200, 40.0 },  // Original
  { 1245, 41.0 },  // Interpolated
  { 1288, 42.0 },  // Interpolated
  { 1330, 43.0 },  // Original
  { 1380, 45.0 },  // Interpolated
  { 1430, 47.0 },  // Interpolated
  { 1480, 49.0 },  // Interpolated
  { 1590, 50.0 },  // Original

  // High range (50-100 PSI) - extrapolated with conservative slope
  { 1680, 55.0 },   // Extrapolated
  { 1780, 60.0 },   // Extrapolated
  { 1880, 65.0 },   // Extrapolated
  { 1980, 70.0 },   // Extrapolated
  { 2080, 75.0 },   // Extrapolated
  { 2180, 80.0 },   // Extrapolated
  { 2280, 85.0 },   // Extrapolated
  { 2380, 90.0 },   // Extrapolated
  { 2480, 95.0 },   // Extrapolated
  { 2580, 100.0 },  // Extrapolated
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
const uint8_t V = 0b00101010;      // V: Segments f, d, b
const uint8_t empty = 0b00000000;  // Empty: No segments

// Variables to track state
int mode = 0;        // Mode: 0 = "Bar", 1 = "PSI", 2 = "mlV"
int brightness = 0;  // Initial brightness level

// Button state variables for single button handling
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
bool longPressHandled = false;

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
      while (1) delay(1000);
    }
  }

  // Use GAIN_ONE for better range up to 4.096V (safer for 4.5V max)
  ads.setGain(GAIN_ONE);  // +/- 4.096V, 1 bit = 0.125mV (good for 0.5-4.5V range)
  ads.setDataRate(128);   // Faster rate for quicker response (was 16)

  delay(500);  // Longer stabilization

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Initialize button pin with internal pull-up

  resetValue();
  display.setBrightness(brightness);  // Set initial brightness level
  updateDisplayMode();
  modeDisplayTime = millis();  // Start timer
}

void loop() {
  // Handle single button press/hold
  handleButtonInput();

  // Switch to selected function after 1.5 seconds
  if (millis() - modeDisplayTime > 1500) {
    switch (mode) {
      case 0:
        pressureMode(false);  // Bar mode
        break;
      case 1:
        pressureMode(true);  // PSI mode
        break;
      case 2:
        rawValueMode();  // milli voltage mode
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
  // Handle negative values
  if (millivolts < 0) return 0.0;

  // Handle values below minimum calibration point
  if (millivolts <= calibrationTable[0].millivolts) {
    // Linear extrapolation from first two points
    float slope = (calibrationTable[1].psi - calibrationTable[0].psi) / (calibrationTable[1].millivolts - calibrationTable[0].millivolts);
    float result = calibrationTable[0].psi + slope * (millivolts - calibrationTable[0].millivolts);
    return (result < 0.0) ? 0.0 : result;
  }

  // Handle values above maximum calibration point
  if (millivolts >= calibrationTable[calibrationPoints - 1].millivolts) {
    // Linear extrapolation from last two points (with limit)
    int lastIdx = calibrationPoints - 1;
    float slope = (calibrationTable[lastIdx].psi - calibrationTable[lastIdx - 1].psi) / (calibrationTable[lastIdx].millivolts - calibrationTable[lastIdx - 1].millivolts);
    float result = calibrationTable[lastIdx].psi + slope * (millivolts - calibrationTable[lastIdx].millivolts);

    // Cap at 110 PSI for safety
    return (result > 110.0) ? 110.0 : result;
  }

  // Linear interpolation between calibration points
  for (int i = 0; i < calibrationPoints - 1; i++) {
    if (millivolts >= calibrationTable[i].millivolts && millivolts <= calibrationTable[i + 1].millivolts) {
      float slope = (calibrationTable[i + 1].psi - calibrationTable[i].psi) / (calibrationTable[i + 1].millivolts - calibrationTable[i].millivolts);
      return calibrationTable[i].psi + slope * (millivolts - calibrationTable[i].millivolts);
    }
  }

  return 0.0;  // Fallback
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

// Function to handle single button input (short press vs long press)
void handleButtonInput() {
  static unsigned long lastDebounceTime = 0;
  static bool lastButtonState = HIGH;
  static bool stableButtonState = HIGH;

  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Debounce logic
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {  // 50ms debounce delay
    if (currentButtonState != stableButtonState) {
      stableButtonState = currentButtonState;

      // Button pressed (HIGH to LOW transition)
      if (stableButtonState == LOW && !buttonPressed) {
        buttonPressed = true;
        buttonPressTime = millis();
        longPressHandled = false;
      }

      // Button released (LOW to HIGH transition)
      if (stableButtonState == HIGH && buttonPressed) {
        buttonPressed = false;
        unsigned long pressDuration = millis() - buttonPressTime;

        // Short press (less than 500ms) - Change brightness
        if (pressDuration < 500 && !longPressHandled) {
          brightness = (brightness + 1) % 8;  // Cycle through brightness levels (wrap from 7 to 0)
          display.setBrightness(brightness);
          updateDisplayMode();
          modeDisplayTime = millis();  // Reset timer
        }
      }
    }
  }

  // Check for long press while button is still pressed
  if (buttonPressed && !longPressHandled && (millis() - buttonPressTime) >= 500) {
    // Long press detected - Change mode
    mode = (mode + 1) % 3;  // Cycle through modes
    updateDisplayMode();
    modeDisplayTime = millis();  // Reset timer
    longPressHandled = true;     // Prevent multiple triggers
  }

  lastButtonState = currentButtonState;
}

// Function to update display mode based on the current mode value
void updateDisplayMode() {
  switch (mode) {
    case 0:
      display.setSegments(new uint8_t[4]{ empty, B, a, r });  // Display "Bar"
      break;
    case 1:
      display.setSegments(new uint8_t[4]{ empty, P, S, I });  // Display "PSI"
      break;
    case 2:
      display.setSegments(new uint8_t[4]{ empty, M, L, V });  // Display "mlV"
      break;
  }
}

void resetValue() {
  mode = 0;
  brightness = 0;
  filteredVoltage = 0;  // Reset filtered voltage
}
