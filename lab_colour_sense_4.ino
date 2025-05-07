#include <Wire.h>
#include <Adafruit_APDS9960.h>

// I2C multiplexer address
#define PCAADDR 0x70

// Output pins for heat control and toast readiness
#define TOAST_READY_PIN 7
#define HEAT_PIN_1 3
#define HEAT_PIN_2 4
#define HEAT_PIN_3 5
#define HEAT_PIN_4 6

// Initialize four APDS-9960 color sensors
Adafruit_APDS9960 sensor1, sensor2, sensor3, sensor4;

// Variables for toast shade and timer control
int shadeIndex = -1;
bool shadeReceived = false;
bool timerStarted = false;
unsigned long startTime = 0;
unsigned long toastDuration = 0;

// Select a specific sensor channel on the PCA9546 I2C multiplexer
void pcaselect(uint8_t i) {
  if (i > 3) return;
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);  // Activate specific channel
  Wire.endTransmission();
}

// Normalize RGB value from 0–255 to 0–1 range
float normalizeRGB(int color) {
  return color / 255.0;
}

// Convert RGB to CIE XYZ color space
void RGBtoXYZ(float r, float g, float b, float &X, float &Y, float &Z) {
  r = (r > 0.04045) ? pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
  g = (g > 0.04045) ? pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
  b = (b > 0.04045) ? pow((b + 0.055) / 1.055, 2.4) : b / 12.92;

  // Linear transformation
  X = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
  Y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
  Z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;
}

// Convert XYZ to CIE L*a*b* color space
void XYZtoLAB(float X, float Y, float Z, float &L, float &a, float &b) {
  float Xn = 0.95047, Yn = 1.00000, Zn = 1.08883;
  float Xr = X / Xn, Yr = Y / Yn, Zr = Z / Zn;

  Xr = (Xr > 0.008856) ? pow(Xr, 1.0 / 3.0) : (903.3 * Xr + 16) / 116;
  Yr = (Yr > 0.008856) ? pow(Yr, 1.0 / 3.0) : (903.3 * Yr + 16) / 116;
  Zr = (Zr > 0.008856) ? pow(Zr, 1.0 / 3.0) : (903.3 * Zr + 16) / 116;

  L = (116 * Yr) - 16;
  a = 500 * (Xr - Yr);
  b = 200 * (Yr - Zr);
}

// Initialization
void setup() {
  Serial.begin(9600);
  Wire.begin(8); // I2C slave address
  Wire.onReceive(receiveEvent); // Register I2C receive callback

  // Set output pins for heater and toast-ready indicator
  pinMode(TOAST_READY_PIN, OUTPUT);
  digitalWrite(TOAST_READY_PIN, HIGH); // High = not ready

  digitalWrite(HEAT_PIN_1, HIGH);
  digitalWrite(HEAT_PIN_2, HIGH);
  digitalWrite(HEAT_PIN_3, HIGH);
  digitalWrite(HEAT_PIN_4, HIGH);

  // Initialize all color sensors via multiplexer
  Adafruit_APDS9960* sensors[] = {&sensor1, &sensor2, &sensor3, &sensor4};
  for (int i = 0; i < 4; i++) {
    pcaselect(i);
    if (!sensors[i]->begin()) {
      Serial.print("Error initializing APDS-9960 sensor ");
      Serial.println(i + 1);
    } else {
      Serial.print("APDS-9960 sensor ");
      Serial.print(i + 1);
      Serial.println(" initialized.");
      sensors[i]->enableColor(true); // Turn on RGB sensing
    }
  }
}

// Main loop: reads from three sensors and checks timer
void loop() {
  readSensor(sensor1, 1);
  readSensor(sensor2, 2);
  readSensor(sensor3, 3);

  checkTimer();

  delay(1000); // Poll every second
}

// Called when a shade index is sent from master (e.g. Raspberry Pi)
void receiveEvent(int howMany) {
  while (Wire.available()) {
    shadeIndex = Wire.read();
    Serial.print("Received Shade Index: ");
    Serial.println(shadeIndex);
    shadeReceived = true;

    // Map shade index to toasting duration
    if (shadeIndex == 0) {
      toastDuration = 180000; // 3 minutes
    } else if (shadeIndex == 1) {
      toastDuration = 300000; // 5 minutes
    } else if (shadeIndex == 2) {
      toastDuration = 480000; // 8 minutes
    }
  }
}

// Reads color from one sensor, converts to LAB, starts timer if toast detected
void readSensor(Adafruit_APDS9960 &sensor, uint8_t sensorID) {
  uint16_t r, g, b, c;

  pcaselect(sensorID - 1); // Activate sensor via I2C mux
  sensor.getColorData(&r, &g, &b, &c); // Read RGB

  float rNorm = normalizeRGB(r);
  float gNorm = normalizeRGB(g);
  float bNorm = normalizeRGB(b);

  float X, Y, Z;
  RGBtoXYZ(rNorm, gNorm, bNorm, X, Y, Z); // Convert to XYZ

  float L, a, bLab;
  XYZtoLAB(X, Y, Z, L, a, bLab); // Convert to LAB

  // Debug output
  Serial.println("L: ");
  Serial.print(L);
  Serial.println("a: ");
  Serial.print(a);
  Serial.println("B: ");
  Serial.print(bLab);

  // If toast is detected (based on lightness), start timer
  if (L > 10 && !timerStarted) {
    startTime = millis();
    timerStarted = true;
    Serial.println("Timer started!");
  }
}

// Checks if the toasting duration has passed
void checkTimer() {
  if (!timerStarted || toastDuration == 0) return;

  unsigned long elapsedTime = millis() - startTime;
  Serial.println(elapsedTime);
  
  // If toasting is done, turn off heating and indicate readiness
  if (elapsedTime >= toastDuration) {
    Serial.println("Toasting time reached!");
    digitalWrite(HEAT_PIN_1, LOW);
    digitalWrite(HEAT_PIN_2, LOW);
    digitalWrite(HEAT_PIN_3, LOW);
    digitalWrite(HEAT_PIN_4, LOW);
    digitalWrite(TOAST_READY_PIN, LOW); // Signal toast is ready
    timerStarted = false;
  }
}
