//#define VERSION "IBUS Telemetry V1.1 20251018 Arduino Pro Mini ATMEGA328 Mini 3.3V 8MHz, BMe/BMP280 3.3V NEO-6M-7M-GPS 5V"
#define VERSION "IBUS Telemetry V1.1 20251018 Arduino Nano ATMEGA328 Mini 5V 16MHz, BMe/BMP280 3.3V NEO-6M-7M-GPS 5V"
#include "iBUSTelemetry.h"
#include <Wire.h>
#include <BME280I2C.h>          // https://github.com/Seeed-Studio/Grove_BME280
#include <TinyGPS++.h>
//#define ALL_SENSORS
#define IBUS_PIN 11             // pin RX IBUS (PCINT na Nano)
iBUSTelemetry telemetry(IBUS_PIN);

BME280I2C bme;                  // default addr 0x76
float temperature = 0, pressure = 0, altitude = 0;
float basePressure = 1013.25;

// --- Hardware state flags ---
bool bmeAvailable = false;
bool bmeHasHumidity = false;
bool gpsAvailable = false;
uint8_t bmeType = 0;   // 0=none, 1=BMP280, 2=BME280, 3=unknown

#define gpsSerial Serial
TinyGPSPlus gps;

#define VOLTAGE_PIN A0
#define VOLTAGE_DIVIDER 23

#define DEBUG_PIN 10             // podłączony do GND = włącz debug
#define UPDATE_INTERVAL 500
uint32_t prevMillis = 0;

double homeLat = 0, homeLon = 0, homeAlt = 0;
bool homeSet = false;

void setup()
{
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  gpsSerial.begin(9600);
  delay(2000); // czekaj na stabilizację zasilania
  if (digitalRead(DEBUG_PIN) == LOW) {
    Serial.println(F(VERSION));
  }
  Wire.begin();
  telemetry.begin();

  // Czujniki 2B
  telemetry.addSensor(IBUS_MEAS_TYPE_TEM);         // 0x01 1
  telemetry.addSensor(IBUS_MEAS_TYPE_EXTV);        // 0x03 2
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_STATUS);  // 0x0B 3
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_DIST);    // 0x14 4
  telemetry.addSensor(IBUS_MEAS_TYPE_CMP_HEAD);    // 0x08 5
  telemetry.addSensor(IBUS_MEAS_TYPE_GROUND_SPEED);// 0x13 6
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_FULL);	    //0xfd 7
  // Czujniki 4B
  telemetry.addSensor(IBUS_MEAS_TYPE_ALT);         // 0x83 (4B) 8
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LAT);     // 0x80  9
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LON);     // 0x81  10
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_ALT);     // 0x82  11
  telemetry.addSensor(IBUS_MEAS_TYPE_HEADING);     // 0x7d Heading 12
  telemetry.addSensor(IBUS_MEAS_TYPE_SPE);       // 0x7e Speed km/h 13
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_DIST);    // 0x14 14

  // --- Initialize BME/BMP sensor with retry limit ---
  const uint8_t maxRetries = 5;
  for (uint8_t i = 0; i < maxRetries; i++) {
    if (bme.begin()) {
      bmeAvailable = true;
      bme.read(pressure, temperature, altitude);
      basePressure = pressure;

      // Identify sensor model
      BME280::ChipModel model = bme.chipModel();
      if (model == BME280::ChipModel_BME280) {
        bmeType = 2;
        bmeHasHumidity = true;
      } else if (model == BME280::ChipModel_BMP280) {
        bmeType = 1;
        bmeHasHumidity = false;
      } else {
        bmeType = 3;
        bmeHasHumidity = false;
      }
      break;
    }
    delay(500);
  }

  // --- GPS detection (2 s timeout) ---
  uint32_t start = millis();
  while (millis() - start < 2000) {
    if (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        gpsAvailable = true;
        break;
      }
    }
  }

  // --- Debug report ---
  if (digitalRead(DEBUG_PIN) == LOW) {
    Serial.println();
    Serial.println(F("=== Hardware detection ==="));

    if (bmeAvailable) {
      if (bmeType == 2) {
        Serial.println(F("BME280 detected (pressure + temperature + humidity)"));
      } else if (bmeType == 1) {
        Serial.println(F("BMP280 detected (pressure + temperature only)"));
      } else {
        Serial.println(F("Unknown BME/BMP sensor model"));
      }

      Serial.print(F("Temperature: ")); Serial.print(temperature, 1); Serial.println(F(" °C"));
      Serial.print(F("Pressure: "));    Serial.print(pressure, 1);    Serial.println(F(" hPa"));

      if (bmeHasHumidity) {
        float humidity = NAN;
        bme.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
        if (!isnan(humidity)) {
          Serial.print(F("Humidity: ")); Serial.print(humidity, 1); Serial.println(F(" %"));
        }
      }
    } else {
      Serial.println(F("No BME/BMP sensor detected"));
    }

    Serial.print(F("GPS: "));
    Serial.println(gpsAvailable ? F("detected (NEO-6M/7M OK)") : F("not detected"));

    Serial.println(F("=========================="));
    Serial.println();
  }
}

void loop()
{
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  updateValues();
  telemetry.run();
}

void updateValues()
{
  uint32_t currMillis = millis();
  if (currMillis - prevMillis < UPDATE_INTERVAL) return;
  prevMillis = currMillis;

  // --- BME/BMP ---
  float relAltitude = 0.0;
  if (bmeAvailable) {
    bme.read(pressure, temperature, altitude);
    relAltitude = 44330.0 * (1.0 - pow((pressure / basePressure), 0.1903));
  } else {
    temperature = 0;
    pressure = 0;
  }

  // --- Napięcie ---
  float adc = analogRead(VOLTAGE_PIN);
  float voltage = (adc / 1023.0) * 5.0 * VOLTAGE_DIVIDER;

  // --- GPS i pochodne ---
  uint8_t fix = gps.location.isValid() ? 3 : 0;
  uint8_t sats = gps.satellites.value();
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double gpsAlt = gps.altitude.meters();
  double speedKmph = gps.speed.kmph();
  double speedMps = gps.speed.mps();
  double course = gps.course.deg();

  // Pozycja startowa
  if (!homeSet && gps.location.isValid()) {
    homeLat = lat;
    homeLon = lon;
    homeAlt = gpsAlt;
    homeSet = true;
  }

  // Dystans, kierunek i względna wysokość
  float distance = 0, heading = 0, gpsRelAlt = 0;
  if (homeSet && gps.location.isValid()) {
    distance = gps.distanceBetween(lat, lon, homeLat, homeLon); // metry
    heading = gps.courseTo(homeLat, homeLon, lat, lon);         // stopnie
    gpsRelAlt = gpsAlt - homeAlt;
  }

  // --- TELEMETRIA ---
  telemetry.setSensorValueFP(1, temperature);                 // TEM
  telemetry.setSensorValue(2, (uint16_t)(voltage * 100));     // EXT_V
  telemetry.setSensorValue(3, telemetry.gpsStateValues(fix, sats)); // GPS_STATUS
  telemetry.setSensorValue(4, (uint16_t)distance);             // GPS_DIST
  telemetry.setSensorValue(5, (uint16_t)heading);              // CMP_HEAD
#ifdef ALL_SENSORS
  telemetry.setSensorValueFP(6, speedMps);                     // GROUND_SPEED
#endif
   telemetry.setSensorValueFP(7, relAltitude);                  // ALT (BME280)
  telemetry.setSensorValue(8, lat * 1e7);                      // GPS_LAT
  telemetry.setSensorValue(9, lon * 1e7);                      // GPS_LON
#ifdef ALL_SENSORS
  telemetry.setSensorValue(10, gpsAlt);                        // GPS_ALT
  telemetry.setSensorValue(11, (uint16_t)course);              // HEADING
  telemetry.setSensorValue(12, (uint16_t)(speedKmph * 100));   // SPEED km/h
#endif
  telemetry.setSensorValue(4, (uint16_t)distance);             // GPS_DIST

  // --- DEBUG ---
  if (digitalRead(DEBUG_PIN) == LOW) {
    Serial.print(F("Tmp1: ")); Serial.print(temperature); Serial.print(F(" C"));
    Serial.print(F(", A3: ")); Serial.print(voltage); Serial.print(F(" V"));
    Serial.print(F(", Pres: ")); Serial.print(pressure); Serial.print(F(" hPa"));
    Serial.print(F(", Sats: ")); Serial.print(sats);
    Serial.print(F(", Fix: ")); Serial.print(fix);
    Serial.print(F(", Dist: ")); Serial.print(distance); Serial.print(F(" m"));
    Serial.print(F(", Hdg: ")); Serial.print(heading); Serial.print(F(" deg (to home)"));
    Serial.print(F(", Course: ")); Serial.print(course); Serial.print(F(" deg (absolute)"));
    Serial.print(F(", SpeedMps: ")); Serial.print(speedMps); Serial.print(F(" m/s"));
    Serial.print(F(", SpeedKmph: ")); Serial.print(speedKmph); Serial.print(F(" km/h"));
    Serial.print(F(", RelAlt: ")); Serial.print(relAltitude); Serial.print(F(" m (BME280)"));
    Serial.print(F(", GPSRelAlt: ")); Serial.print(gpsRelAlt); Serial.print(F(" m"));
    Serial.print(F(", Lat: ")); Serial.print(lat, 7);
    Serial.print(F(", Lon: ")); Serial.print(lon, 7);
    Serial.print(F(", GPSAlt: ")); Serial.print(gpsAlt); Serial.print(F(" m"));
    Serial.print(homeSet ? F(", Home SET") : F(", Home NOT SET"));
    Serial.print(F(", homeLat: ")); Serial.print(homeLat, 7);
    Serial.print(F(", homeLon: ")); Serial.print(homeLon, 7);
    Serial.print(F(", homeAlt: ")); Serial.print(homeAlt); Serial.print(F(" m"));
    Serial.println();
  }
}
