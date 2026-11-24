/* Automated Vermicast Production Machine
*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

constexpr int MOISTURE_PIN = 34;
constexpr int TEMP_PIN = 4;
constexpr int SOLENOID_RELAY_PIN = 26;
constexpr int PUMP_RELAY_PIN = 27; // FAA dosing pump
constexpr int RE_DE_PIN = 14;
constexpr int RO_PIN = 16; // ESP32 RX2
constexpr int DI_PIN = 17; // ESP32 TX2
constexpr int MOIST_LOWER_THRESH = 60;
constexpr int MOIST_UPPER_THRESH = 65;
constexpr float TEMP_MAX_THRESH = 33.0f;
constexpr unsigned long READ_INTERVAL_MS = 5000;
constexpr unsigned long MQTT_RETRY_MS = 5000;
constexpr unsigned long COOLING_PULSE_MS = 2000;
constexpr int ADC_DRY = 3000; // Calibrate with actual sensor
constexpr int ADC_WET = 1300; // Calibrate with actual sensor

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST = "YOUR_MQTT_BROKER";
constexpr uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWORD = "";
const char* MQTT_TOPIC = "vermicast/telemetry";

LiquidCrystal_I2C lcd(0x27, 20, 4);
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
HardwareSerial npkSerial(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Additional relays and physical controls (proposed pins; verify wiring).
constexpr int SHREDDER_RELAY_PIN = 25;
constexpr int SIFTER_RELAY_PIN = 32;
constexpr int SHREDDER_BUTTON_PIN = 18; // NO pushbutton to GND
constexpr int SIFTER_BUTTON_PIN = 19;   // NO pushbutton to GND
constexpr int FAA_BUTTON_PIN = 23;      // NO pushbutton to GND
constexpr int ESTOP_OK_PIN = 13;        // NC safety contact to GND; HIGH = stop
constexpr unsigned long SHREDDER_RUN_MS = 15000UL;
constexpr unsigned long SIFTER_RUN_MS = 20000UL;
constexpr unsigned long FAA_DOSE_MS = 3000UL; // MUST calibrate flow rate and dosage
constexpr unsigned long FAA_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 80UL;
// Automatic FAA dosing is disabled until a safe dose and schedule are validated.
constexpr bool ENABLE_AUTOMATIC_FAA = false;
constexpr bool RELAY_ACTIVE_LOW = true;

bool isWatering = false;
bool coolingPulse = false;
unsigned long coolingStarted = 0;
unsigned long lastReadTime = 0;
unsigned long lastMqttAttempt = 0;

// Report's Modbus request: slave 1, function 03, start register 0x001E, 3 registers.
// Validate address and CRC against the actual NPK probe datasheet.
const uint8_t NPK_REQUEST[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xCD};

uint16_t modbusCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

bool readNPK(uint16_t &n, uint16_t &p, uint16_t &k) {
  while (npkSerial.available()) npkSerial.read();
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(200);
  npkSerial.write(NPK_REQUEST, sizeof(NPK_REQUEST));
  npkSerial.flush();
  delayMicroseconds(200);
  digitalWrite(RE_DE_PIN, LOW);

  uint8_t reply[11];
  size_t count = 0;
  unsigned long start = millis();
  while (count < sizeof(reply) && millis() - start < 500) {
    if (npkSerial.available()) reply[count++] = (uint8_t)npkSerial.read();
    else delay(1);
  }
  if (count != sizeof(reply) || reply[0] != 1 || reply[1] != 3 || reply[2] != 6)
    return false;
  uint16_t crc = modbusCRC(reply, 9);
  if (reply[9] != (crc & 0xFF) || reply[10] != (crc >> 8)) return false;
  n = ((uint16_t)reply[3] << 8) | reply[4];
  p = ((uint16_t)reply[5] << 8) | reply[6];
  k = ((uint16_t)reply[7] << 8) | reply[8];
  return true;
}

void setValve(bool on) {
  isWatering = on;
  digitalWrite(SOLENOID_RELAY_PIN, on ? LOW : HIGH); // Active-low relay
}

void updateLCD(int moisture, float temp, bool tempValid, bool npkValid,
               uint16_t n, uint16_t p, uint16_t k) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moist:"); lcd.print(moisture); lcd.print("% T:");
  if (tempValid) lcd.print(temp, 1); else lcd.print("ERR");
  lcd.setCursor(0, 1);
  lcd.print("Valve:"); lcd.print(isWatering ? "OPEN" : "CLOSED");
  lcd.setCursor(0, 2);
  if (npkValid) { lcd.print("N:"); lcd.print(n); lcd.print(" P:"); lcd.print(p); }
  else lcd.print("NPK sensor: no data");
  lcd.setCursor(0, 3);
  if (npkValid) { lcd.print("K:"); lcd.print(k); lcd.print(" mg/kg"); }
  else lcd.print("Check RS485 wiring");
}

void ensureConnections() {
  if (strcmp(MQTT_HOST, "YOUR_MQTT_BROKER") == 0 || WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();
  String clientId = "vermicast-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (strlen(MQTT_USER)) mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
  else mqtt.connect(clientId.c_str());
}

void publishTelemetry(int moisture, float temp, bool tempValid,
                      bool npkValid, uint16_t n, uint16_t p, uint16_t k) {
  if (!mqtt.connected()) return;
  char payload[256];
  char temperature[16];
  if (tempValid) snprintf(temperature, sizeof(temperature), "%.2f", temp);
  else snprintf(temperature, sizeof(temperature), "null");
  snprintf(payload, sizeof(payload),
           "{\"moisture\":%d,\"temperature\":%s,\"watering\":%s,"
           "\"npk_valid\":%s,\"nitrogen\":%u,\"phosphorus\":%u,\"potassium\":%u}",
           moisture, temperature, isWatering ? "true" : "false",
           npkValid ? "true" : "false", n, p, k);
  mqtt.publish(MQTT_TOPIC, payload);
}


// AC shredder/sifter must be driven by correctly rated contactors, not directly
// from ESP32 pins or an undersized hobby relay. E-stop MUST interrupt motor power
// through a properly designed hardwired safety circuit independently of firmware.
void relayWrite(int pin, bool on) {
  digitalWrite(pin, (on == RELAY_ACTIVE_LOW) ? LOW : HIGH);
}

bool shredderRunning = false, sifterRunning = false, faaRunning = false;
unsigned long shredderStarted = 0, sifterStarted = 0, faaStarted = 0;
unsigned long lastFaaStarted = 0;
bool faaEverStarted = false;
bool lastShredderButton = HIGH, lastSifterButton = HIGH, lastFaaButton = HIGH;
unsigned long lastShredderPress = 0, lastSifterPress = 0, lastFaaPress = 0;

void stopProcessMotors() {
  shredderRunning = sifterRunning = faaRunning = false;
  relayWrite(SHREDDER_RELAY_PIN, false);
  relayWrite(SIFTER_RELAY_PIN, false);
  relayWrite(PUMP_RELAY_PIN, false);
}

bool safetyOK() { return digitalRead(ESTOP_OK_PIN) == LOW; }

void startShredder(unsigned long now) {
  if (!safetyOK() || sifterRunning || faaRunning || shredderRunning) return;
  shredderRunning = true;
  shredderStarted = now;
  relayWrite(SHREDDER_RELAY_PIN, true);
}
void startSifter(unsigned long now) {
  if (!safetyOK() || shredderRunning || faaRunning || sifterRunning) return;
  sifterRunning = true;
  sifterStarted = now;
  relayWrite(SIFTER_RELAY_PIN, true);
}
void startFAA(unsigned long now) {
  if (!safetyOK() || shredderRunning || sifterRunning || faaRunning) return;
  faaRunning = true;
  faaStarted = lastFaaStarted = now;
  faaEverStarted = true;
  relayWrite(PUMP_RELAY_PIN, true);
}

void handleButtonsAndMotors(unsigned long now) {
  if (!safetyOK()) {
    stopProcessMotors();
    setValve(false);
    return;
  }
  if (shredderRunning && now - shredderStarted >= SHREDDER_RUN_MS) {
    shredderRunning = false; relayWrite(SHREDDER_RELAY_PIN, false);
  }
  if (sifterRunning && now - sifterStarted >= SIFTER_RUN_MS) {
    sifterRunning = false; relayWrite(SIFTER_RELAY_PIN, false);
  }
  if (faaRunning && now - faaStarted >= FAA_DOSE_MS) {
    faaRunning = false; relayWrite(PUMP_RELAY_PIN, false);
  }
  bool sh = digitalRead(SHREDDER_BUTTON_PIN);
  bool si = digitalRead(SIFTER_BUTTON_PIN);
  bool fa = digitalRead(FAA_BUTTON_PIN);
  if (lastShredderButton == HIGH && sh == LOW && now - lastShredderPress > BUTTON_DEBOUNCE_MS) {
    lastShredderPress = now; startShredder(now);
  }
  if (lastSifterButton == HIGH && si == LOW && now - lastSifterPress > BUTTON_DEBOUNCE_MS) {
    lastSifterPress = now; startSifter(now);
  }
  if (lastFaaButton == HIGH && fa == LOW && now - lastFaaPress > BUTTON_DEBOUNCE_MS) {
    lastFaaPress = now; startFAA(now);
  }
  lastShredderButton = sh;
  lastSifterButton = si;
  lastFaaButton = fa;
  if (ENABLE_AUTOMATIC_FAA &&
      ((!faaEverStarted && now >= FAA_INTERVAL_MS) ||
       (faaEverStarted && now - lastFaaStarted >= FAA_INTERVAL_MS))) {
    startFAA(now);
  }
}

void setup() {
  Serial.begin(115200);
  // Set inactive relay output levels BEFORE configuring pins as outputs.
  const int relays[] = {SOLENOID_RELAY_PIN, PUMP_RELAY_PIN, SHREDDER_RELAY_PIN, SIFTER_RELAY_PIN};
  for (int pin : relays) {
    digitalWrite(pin, RELAY_ACTIVE_LOW ? HIGH : LOW);
    pinMode(pin, OUTPUT);
  }
  pinMode(RE_DE_PIN, OUTPUT); digitalWrite(RE_DE_PIN, LOW);
  pinMode(SHREDDER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SIFTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FAA_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ESTOP_OK_PIN, INPUT_PULLUP);
  stopProcessMotors(); setValve(false);
  Wire.begin(21, 22);
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Vermicast ESP32");
  tempSensor.begin(); tempSensor.setResolution(10);
  npkSerial.begin(4800, SERIAL_8N1, RO_PIN, DI_PIN);
  WiFi.mode(WIFI_STA);
  if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0) WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  Serial.println("Hardware safety, NPK registers, moisture and pump require validation.");
}

void loop() {
  unsigned long now = millis();
  handleButtonsAndMotors(now);
  ensureConnections();
  if (coolingPulse && now - coolingStarted >= COOLING_PULSE_MS) {
    coolingPulse = false; setValve(false);
  }
  if (now - lastReadTime < READ_INTERVAL_MS) return;
  lastReadTime = now;
  int raw = analogRead(MOISTURE_PIN);
  bool moistureValid = (raw > 50 && raw < 4050 && ADC_DRY != ADC_WET);
  int moisture = moistureValid ? constrain(map(raw, ADC_DRY, ADC_WET, 0, 100), 0, 100) : -1;
  tempSensor.requestTemperatures();
  float temp = tempSensor.getTempCByIndex(0);
  bool tempValid = temp != DEVICE_DISCONNECTED_C && temp > -55 && temp < 125;
  uint16_t n = 0, p = 0, k = 0;
  bool npkValid = readNPK(n, p, k);
  if (!safetyOK() || !moistureValid) {
    coolingPulse = false; setValve(false);
  } else if (!coolingPulse) {
    if (moisture < MOIST_LOWER_THRESH) setValve(true);
    else if (moisture >= MOIST_UPPER_THRESH) setValve(false);
  }
  updateLCD(moisture, temp, tempValid, npkValid, n, p, k);
  publishTelemetry(moisture, temp, tempValid, npkValid, n, p, k);
  Serial.printf("Moist=%d%% Temp=%.1f NPK=%d Valve=%d Shred=%d Sift=%d FAA=%d Safety=%d\n",
    moisture, temp, npkValid, isWatering, shredderRunning, sifterRunning, faaRunning, safetyOK());
}
