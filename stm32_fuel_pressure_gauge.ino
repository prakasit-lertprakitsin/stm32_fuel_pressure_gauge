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
  // ช่วง 0-25 PSI (คาดคะแน - extrapolation ย้อนกลับ)
  { 150, 0.0 },   // คาดคะแน: sensor minimum output
  { 250, 5.0 },   // คาดคะแน: linear trend จากจุดแรก
  { 350, 10.0 },  // คาดคะแน
  { 425, 15.0 },  // คาดคะแน: slope เริ่มเพิ่ม
  { 485, 20.0 },  // คาดคะแน: curve เริ่มโค้ง

  // ข้อมูลจริงที่มีอยู่
  { 595, 25.0 },   // ข้อมูลจริง
  { 635, 26.0 },   // ข้อมูลจริง
  { 670, 27.0 },   // ข้อมูลจริง
  { 850, 30.0 },   // ข้อมูลจริง
  { 1035, 35.0 },  // ข้อมูลจริง
  { 1200, 40.0 },  // ข้อมูลจริง

  // ช่วง 40-50 PSI (interpolation จากข้อมูลจริง)
  { 1390, 45.0 },  // คาดคะแน: interpolation ระหว่าง 40-50 PSI
  { 1580, 50.0 },  // ข้อมูลจริง

  // ช่วง 50-100 PSI (คาดคะแน - extrapolation ไปข้างหน้า)
  { 1770, 55.0 },  // คาดคะแน: slope ลดลงเล็กน้อย
  { 1955, 60.0 },  // คาดคะแน: curve เริ่มโค้งมากขึ้น
  { 2135, 65.0 },  // คาดคะแน: slope ลดลงต่อเนื่อง
  { 2310, 70.0 },  // คาดคะแน
  { 2480, 75.0 },  // คาดคะแน: approaching saturation
  { 2645, 80.0 },  // คาดคะแน
  { 2805, 85.0 },  // คาดคะแน: slope ลดลงมาก
  { 2960, 90.0 },  // คาดคะแน
  { 3110, 95.0 },  // คาดคะแน: near saturation
  { 3255, 100.0 }  // คาดคะแน: maximum practical range
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
  // Handle values below minimum
  if (millivolts <= calibrationTable[0].millivolts) {
    if (millivolts < 100) return 0.0;  // Below sensor range

    // Conservative extrapolation
    float slope = (calibrationTable[1].psi - calibrationTable[0].psi) / (calibrationTable[1].millivolts - calibrationTable[0].millivolts);
    slope *= 0.7;  // Reduce slope for safety
    float result = calibrationTable[0].psi + slope * (millivolts - calibrationTable[0].millivolts);
    return (result < 0) ? 0.0 : result;
  }

  // Handle values above maximum
  if (millivolts >= calibrationTable[calibrationPoints - 1].millivolts) {
    if (millivolts > 4000) return 100.0;  // Hard limit

    // Use last 3 points for more stable extrapolation
    int lastIdx = calibrationPoints - 1;
    float slope1 = (calibrationTable[lastIdx].psi - calibrationTable[lastIdx - 1].psi) / (calibrationTable[lastIdx].millivolts - calibrationTable[lastIdx - 1].millivolts);
    float slope2 = (calibrationTable[lastIdx - 1].psi - calibrationTable[lastIdx - 2].psi) / (calibrationTable[lastIdx - 1].millivolts - calibrationTable[lastIdx - 2].millivolts);

    float avgSlope = (slope1 + slope2) / 2.0;
    avgSlope *= 0.8;  // Conservative extrapolation

    float result = calibrationTable[lastIdx].psi + avgSlope * (millivolts - calibrationTable[lastIdx].millivolts);

    return (result > 100.0) ? 100.0 : result;
  }

  // Linear interpolation between points
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
