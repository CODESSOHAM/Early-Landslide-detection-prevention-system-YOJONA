#include <Wire.h>  // For I2C communication
#include <Adafruit_Sensor.h>  // Adafruit sensor library
#include <Adafruit_ADXL345_U.h>  // Adafruit ADXL345 library

// Pin Definitions
const int vibrationPin = 2;      // Pin for vibration sensor (Earthquake detection)
const int raindropPin = A0;      // Pin for raindrop sensor (Rain detection)
const int soilMoisturePin = A1;  // Pin for soil moisture sensor (Humidity detection)
const int ledPin = 3;            // Pin for LED
const int buzzerPin = 4;         // Pin for buzzer

// ADXL345 object
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);  // Unique sensor ID

// Variables
int vibrationState = 0;         // State of vibration sensor
int raindropValue = 0;          // Value from raindrop sensor
int soilMoistureValue = 0;      // Value from soil moisture sensor
bool isRainDetected = false;    // Flag for rain detection
bool isSoilMoist = false;       // Flag for soil moisture detection
bool isEarthquakeDetected = false; // Flag for earthquake detection
bool isSoilMovementDetected = false; // Flag for soil movement detection

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);

  // Set up sensor and output pins
  pinMode(vibrationPin, INPUT);
  pinMode(raindropPin, INPUT);
  pinMode(soilMoisturePin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Initialize LED and buzzer to OFF
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Initialize ADXL345
  if (!adxl.begin()) {
    Serial.println("Could not find ADXL345. Check wiring!");
    while (1);  // Halt if sensor is not found
  }
  Serial.println("ADXL345 initialized.");

  // Set range and sensitivity
  adxl.setRange(ADXL345_RANGE_16_G);  // Set range to ±16g
  adxl.setDataRate(ADXL345_DATARATE_100_HZ);  // Set data rate to 100Hz

  Serial.println("Landslide Detection System Initialized");
  Serial.println("Monitoring for landslide conditions...");
}

void loop() {
  // Read sensor values
  vibrationState = digitalRead(vibrationPin);  // Earthquake detection
  raindropValue = analogRead(raindropPin);     // Rain detection
  soilMoistureValue = analogRead(soilMoisturePin); // Soil moisture detection

  // Read accelerometer data for soil movement detection
  sensors_event_t event;
  adxl.getEvent(&event);
  float totalAccel = sqrt(event.acceleration.x * event.acceleration.x +
                          event.acceleration.y * event.acceleration.y +
                          event.acceleration.z * event.acceleration.z);

  // Determine conditions
  isRainDetected = (raindropValue < 500);  // Rain detected if value < 500
  isSoilMoist = (soilMoistureValue > 700); // Excess soil moisture if value > 700
  isEarthquakeDetected = (vibrationState == HIGH); // Earthquake detected if vibration is HIGH
  isSoilMovementDetected = (totalAccel > 11); // Soil movement detected if acceleration > 11 m/s²

  // Calculate landslide chance based on conditions
  int landslideChance = 0;

  if (isRainDetected && !isSoilMoist && !isEarthquakeDetected && !isSoilMovementDetected) {
    landslideChance = random(15, 21);  // 15-20%
  } else if (!isRainDetected && isSoilMoist && !isEarthquakeDetected && !isSoilMovementDetected) {
    landslideChance = random(10, 16);  // 10-15%
  } else if (isRainDetected && isSoilMoist && !isEarthquakeDetected && !isSoilMovementDetected) {
    landslideChance = random(25, 31);  // 25-30%
  } else if (!isRainDetected && !isSoilMoist && isEarthquakeDetected && !isSoilMovementDetected) {
    landslideChance = random(40, 46);  // 40-45%
  } else if (isRainDetected && isSoilMoist && isEarthquakeDetected && !isSoilMovementDetected) {
    landslideChance = random(50, 56);  // 50-55%
  } else if (!isRainDetected && !isSoilMoist && !isEarthquakeDetected && isSoilMovementDetected) {
    landslideChance = random(70, 76);  // 70-75%
  } else if (!isRainDetected && !isSoilMoist && isEarthquakeDetected && isSoilMovementDetected) {
    landslideChance = random(75, 86);  // 75-85%
  } else if (isRainDetected && isSoilMoist && isEarthquakeDetected && isSoilMovementDetected) {
    landslideChance = random(87, 98);  // 87-97%
  }

  // Print landslide chance to Serial Monitor
  Serial.print("Landslide Chance: ");
  Serial.print(landslideChance);
  Serial.println("%");

  // Small delay to avoid rapid toggling
  delay(1000);  // Update every 1 second
}
