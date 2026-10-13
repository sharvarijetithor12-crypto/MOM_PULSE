
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "MAX30105.h"
#include "heartRate.h"

// Wi-Fi
const char* AP_SSID = "MomPulse";
const char* AP_PASS = "12345678";

// Pins
const int SOS_BUTTON = 0;
const int GPS_RX_PIN = 18;
const int GPS_TX_PIN = 17;
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Demo location: used only if GPS has no valid recent fix
const double DEMO_LAT = 19.0760;
const double DEMO_LON = 72.8777;

// Risk thresholds
const int BPM_LOW = 20;
const int BPM_HIGH = 120;
const int ABNORMAL_REQUIRED = 3;

const unsigned long AUTO_SOS_GAP = 60000;
const unsigned long DEBOUNCE_MS = 50;

// Objects
WebServer server(80);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
MAX30105 particleSensor;

// States
bool sensorOK = false;
bool sosActive = false;
bool fingerOn = false;
bool sosIsAutomatic = false;
bool demoLocation = true;

double sosLat = 0;
double sosLon = 0;

// Heart-rate averaging
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE] = {0};
byte rateSpot = 0;
byte beatCount = 0;

unsigned long lastBeat = 0;
unsigned long lastAutoSos = 0;

int beatAvg = 0;
int abnormalCount = 0;

// Button debounce
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// ------------------------------------------------
// Trigger SOS
// ------------------------------------------------
void triggerSOS(bool automaticSOS) {
  if (sosActive) return;

  sosIsAutomatic = automaticSOS;

  if (gps.location.isValid() &&
      gps.location.age() < 10000) {
    sosLat = gps.location.lat();
    sosLon = gps.location.lng();
    demoLocation = false;
  } else {
    sosLat = DEMO_LAT;
    sosLon = DEMO_LON;
    demoLocation = true;
  }

  sosActive = true;

  Serial.println(automaticSOS
                 ? "AUTOMATIC RISK SOS!"
                 : "MANUAL SOS!");

  Serial.printf("BPM: %d\n", beatAvg);
  Serial.printf("Location: %.6f, %.6f\n", sosLat, sosLon);

  if (demoLocation) {
    Serial.println("WARNING: Demo location; GPS fix unavailable.");
  }
}

// ------------------------------------------------
// Status endpoint
// Maintains original plain-text format for APK:
// normal = BPM number
// SOS = latitude,longitude
// ------------------------------------------------
void handleStatus() {
  if (sosActive) {
    server.send(
      200,
      "text/plain",
      String(sosLat, 6) + "," + String(sosLon, 6)
    );
  } else {
    int shown = (sensorOK && fingerOn) ? beatAvg : 0;
    server.send(200, "text/plain", String(shown));
  }
}

// ------------------------------------------------
// Acknowledge SOS
// ------------------------------------------------
void handleAck() {
  sosActive = false;
  sosIsAutomatic = false;
  abnormalCount = 0;

  server.send(200, "text/plain", "ok");
  Serial.println("SOS acknowledged by app");
}

// ------------------------------------------------
// Heart rate
// ------------------------------------------------
void readHeartRate() {
  if (!sensorOK) return;

  long irValue = particleSensor.getIR();

  fingerOn = (irValue > 50000);

  if (!fingerOn) {
    beatCount = 0;
    rateSpot = 0;
    beatAvg = 0;
    abnormalCount = 0;
    lastBeat = 0;
    return;
  }

  if (checkForBeat(irValue)) {
    unsigned long now = millis();

    if (lastBeat != 0) {
      unsigned long delta = now - lastBeat;

      if (delta > 0) {
        float currentBPM = 60000.0 / delta;

        if (currentBPM >= 20 && currentBPM <= 75) {
          rates[rateSpot] = (byte)currentBPM;
          rateSpot = (rateSpot + 1) % RATE_SIZE;

          if (beatCount < RATE_SIZE) {
            beatCount++;
          }

          int sum = 0;

          for (byte i = 0; i < beatCount; i++) {
            sum += rates[i];
          }

          beatAvg = sum / beatCount;

          Serial.printf("BPM: %d\n", beatAvg);

          // Only evaluate risk after enough valid beats
          if (beatCount >= RATE_SIZE) {
            if (beatAvg < BPM_LOW ||
                beatAvg > BPM_HIGH) {
              abnormalCount++;
            } else {
              abnormalCount = 0;
            }

            if (abnormalCount >= ABNORMAL_REQUIRED &&
                !sosActive &&
                millis() - lastAutoSos >= AUTO_SOS_GAP) {

              lastAutoSos = millis();
              abnormalCount = 0;
              triggerSOS(true);
            }
          }
        }
      }
    }

    lastBeat = now;
  }

  // Clear stale heart-rate readings
  if (lastBeat != 0 && millis() - lastBeat > 5000) {
    beatAvg = 0;
    beatCount = 0;
    abnormalCount = 0;
  }
}

// ------------------------------------------------
// Setup
// ------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(SOS_BUTTON, INPUT_PULLUP);

  GPSSerial.begin(9600, SERIAL_8N1,
                  GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    sensorOK = true;

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeIR(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);

    Serial.println("MAX30102 OK");
  } else {
    sensorOK = false;
    Serial.println("MAX30102 not detected. Check wiring.");
  }

  WiFi.mode(WIFI_AP);

  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.print("Wi-Fi started. IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Wi-Fi start failed");
  }

  server.on("/status", HTTP_GET, handleStatus);
  server.on("/ack", HTTP_GET, handleAck);

  server.begin();
  Serial.println("Mom Pulse ready");
}

// ------------------------------------------------
// Main loop
// ------------------------------------------------
void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  readHeartRate();
  server.handleClient();

  // Debounced BOOT button
  int reading = digitalRead(SOS_BUTTON);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW && !sosActive) {
        triggerSOS(false);
      }
    }
  }

  lastButtonReading = reading;

  // Debug every 5 seconds
  static unsigned long lastDebug = 0;

  if (millis() - lastDebug >= 5000) {
    lastDebug = millis();

    Serial.printf(
      "BPM: %d | Finger: %s | GPS Fix: %s\n",
      beatAvg,
      fingerOn ? "YES" : "NO",
      (gps.location.isValid() &&
       gps.location.age() < 10000) ? "YES" : "NO"
    );
  }
}
```
