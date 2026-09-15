/* 
  ESP32 HQ - LoRa <-> ESP-NOW bridge + Sensor Suite (BH1750, MPU6050, ACS)
  - For ESP32
  - LoRa receives packets from Laptop-LoRa and forwards them to branch nodes via ESP-NOW
  - ESP-NOW receives messages from branch nodes and forwards them via LoRa to Laptop
  - Includes sensor checks and fault reporting (forwards as ESP-NOW and LoRa)
  - Updated with advanced current sensor and pole fall detection logic
  - Modified to continue booting even if hardware is not detected
  - MODIFIED: Pole fall alerts sent directly via LoRa (not ESP-NOW)
  - MODIFIED: Increased sensitivity for pole fall detection
  - MODIFIED: Improved current sensor calibration with 60s stabilization
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BH1750.h>
#include <MPU6050.h>
#include <math.h>

// ---------------- CONFIG / IDs ----------------
#define LORA_ID   4        // LoRa network destination ID for Laptop-bridge
#define HQ_ID     4000     // This HQ's ID used as SRC in forwarded messages
#define SERIAL_BAUD 9600

// ---------------- PINS ----------------
const int BUZZER_PIN = 25;    
const int LED_PIN    = 2;
const int LORA_SS    = 5;
const int LORA_RST   = 15;   // *** CHANGED from 14 to 15 to avoid LED conflict ***
const int LORA_DIO0  = 26;
const long LORA_FREQ = 433E6;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const int ACS_PIN = 33;       // Pin for ACS sensor
const int WIFI_CHANNEL = 1;

// ---------------- ESP-NOW Peers ----------------
uint8_t node1Mac[6] = {0x58,0xBF,0x25,0xDC,0xB9,0xFA};
uint8_t node2Mac[6] = {0x58,0xBF,0x25,0xDC,0xB9,0xFA};
uint8_t *peers[] = { node1Mac, node2Mac };
const int NUM_PEERS = sizeof(peers) / sizeof(peers[0]);

// ---------------- OBJECTS ----------------
BH1750 lightMeter;
MPU6050 mpu;

// ---------------- FLAGS ----------------
bool bh1750_ok = false;
bool mpu_ok   = false;
bool lora_ok  = false;  // Added flag for LoRa status
bool espnow_ok = false; // Added flag for ESP-NOW status

// ---------------- THRESHOLDS ----------------
const float LUX_THRESHOLD        = 300.0;  // Before it was 500
const unsigned long READ_INTERVAL= 1000;
const float TILT_THRESHOLD       = 45.0;
const float CURRENT_THRESHOLD    = 0.055; // It is changing for different ESPs and Different sensor units
const float alpha                = 0.1;
const int SAMPLE_COUNT           = 10;

// Pole fallen detection parameters - INCREASED SENSITIVITY
const float SPEED_CHANGE_THRESHOLD = 0.3;  // DECREASED from 0.8 for higher sensitivity
const float ANGLE_CHANGE_THRESHOLD = 5.0;  // DECREASED from 10.0 for higher sensitivity
const float ABSOLUTE_TILT_THRESHOLD = 40.0; // DECREASED from 55.0 for higher sensitivity
const unsigned long POLE_FALL_COOLDOWN = 30000;
const unsigned long POLE_FALL_CHECK_INTERVAL = 50; // Check every 50ms

// Noise filtering
const float NOISE_THRESHOLD = 0.05; // DECREASED from 0.1 for higher sensitivity
const int STABLE_READINGS = 1; // DECREASED from 2 for faster response

// Current sensor specific variables
unsigned long lastRecalibration = 0;
const unsigned long RECALIBRATION_INTERVAL = 60000;

// Fault generation cooldown
static unsigned long lastFaultGeneration = 0;
const unsigned long FAULT_GENERATION_COOLDOWN = 30000;

// ---------------- GLOBAL VARS ----------------
float sensorOffset     = 0.0;
float filteredCurrent  = 0.0;
bool calibrated        = false;
bool poleFallen        = false;
float baselineX=0.0, baselineY=0.0, baselineZ=1.0;

// For tracking changes in acceleration and angle
float prevAccelX = 0.0, prevAccelY = 0.0, prevAccelZ = 0.0;
float prevAngle = 0.0;
unsigned long lastPoleFallAlert = 0;
unsigned long lastPoleFallCheck = 0;
unsigned long lastMPUCheck = 0;
const unsigned long MPU_RECHECK_INTERVAL = 5000;

// Noise filtering variables
float stableAx = 0.0, stableAy = 0.0, stableAz = 0.0;
int stableCount = 0;

float luxSamples[SAMPLE_COUNT];
bool lightStates[SAMPLE_COUNT];
int sampleIndex = 0;
bool bufferFilled = false;

unsigned long lastCurrentRead = 0;
unsigned long lastSend        = 0;

unsigned long alertStart   = 0;
bool alertActive           = false;
unsigned long alertDurationMs = 0;

// LoRa send cooldown
unsigned long lastLoRaSend = 0;
const unsigned long SEND_COOLDOWN_MS = 100;

// ---------------- FUNCTION DECLARATIONS ----------------
void setupLoRa();
void setupEspNow();
void forwardViaEspNow(const String &payload);
void forwardViaLoRa(const String &payload);
void onLoRaReceive(int packetSize);
void triggerBuzzerFor(unsigned long ms);
void beep(int t);
float getSensorOffset();
float getACCurrent();
void calibrateMPU();
void checkSensors();
void recalibrateCurrentSensor();
void checkPoleFallAsync();
bool initializeMPU();
void sendPoleFallFault();

// ✅ Updated ESP-NOW callback declarations (for ESP-IDF v5.x)
void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("===== ESP32 HQ BOOT =====");
  Serial.printf("LoRa ID = %d | HQ ID = %d\n", LORA_ID, HQ_ID);

  pinMode(BUZZER_PIN, OUTPUT);
  // pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize MPU6050 first
  mpu_ok = initializeMPU();
  
  bh1750_ok = lightMeter.begin();
  if (bh1750_ok) Serial.println("BH1750 OK");
  else Serial.println("BH1750 not detected");

  // Calibrate current sensor - wait 60s for stabilization, then average 10 readings
  Serial.println("Calibrating current sensor... This will take approximately 70 seconds.");
  sensorOffset = getSensorOffset();
  Serial.print("Initial Sensor Offset: "); Serial.println(sensorOffset);
  lastRecalibration = millis();
  
  if (mpu_ok) calibrateMPU();

  setupLoRa();
  setupEspNow();

  // Print system status
  Serial.println("\n===== SYSTEM STATUS =====");
  Serial.printf("LoRa: %s\n", lora_ok ? "OK" : "FAILED");
  Serial.printf("ESP-NOW: %s\n", espnow_ok ? "OK" : "FAILED");
  Serial.printf("BH1750: %s\n", bh1750_ok ? "OK" : "FAILED");
  Serial.printf("MPU6050: %s\n", mpu_ok ? "OK" : "FAILED");
  Serial.println("========================\n");

  Serial.println("System ready.\n");
}

// ---------------- LOOP ----------------
void loop() {
  // Only process LoRa packets if LoRa is initialized
  if (lora_ok) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) onLoRaReceive(packetSize);
  }

  if (alertActive && (millis() - alertStart >= alertDurationMs)) {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    alertActive = false;
    Serial.println("Alert completed, buzzer off.");
  }

  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 3000) {
    lastCheck = millis();
    checkSensors();
  }
  
  // Asynchronous pole fall detection - check every 50ms
  if (mpu_ok && calibrated) {
    checkPoleFallAsync();
  }
  
  // Periodically check MPU connection and try to reconnect if needed
  if (millis() - lastMPUCheck > MPU_RECHECK_INTERVAL) {
    lastMPUCheck = millis();
    if (!mpu_ok) {
      Serial.println("Attempting to reconnect MPU6050...");
      mpu_ok = initializeMPU();
      if (mpu_ok) {
        calibrateMPU();
        Serial.println("MPU6050 reconnected and recalibrated");
      }
    }
  }
  
  // Periodic recalibration to prevent drift
  if (millis() - lastRecalibration > RECALIBRATION_INTERVAL) {
    recalibrateCurrentSensor();
  }
  
  delay(10);
}

// ---------------- LoRa receive handler ----------------
void onLoRaReceive(int packetSize) {
  String packet = "";
  while (LoRa.available()) packet += (char)LoRa.read();
  packet.trim();
  Serial.println("\n[LoRa RX] " + packet);

  String dst = "", tgt = "", dur = "", msg = "";
  int pDst = packet.indexOf("DST:");
  int pTgt = packet.indexOf("|TGT:");
  int pDur = packet.indexOf("|DUR:");
  int pMsg = packet.indexOf("|MSG:");

  if (pDst >= 0 && pMsg > pDst)
    dst = packet.substring(pDst + 4, (pTgt > pDst) ? pTgt : pMsg);
  if (pTgt >= 0 && pMsg > pTgt)
    tgt = packet.substring(pTgt + 5, (pDur > pTgt) ? pDur : pMsg);
  if (pDur >= 0 && pMsg > pDur)
    dur = packet.substring(pDur + 5, pMsg);
  if (pMsg >= 0)
    msg = packet.substring(pMsg + 5);

  dst.trim(); tgt.trim(); dur.trim(); msg.trim();

  int targetLoraId = dst.toInt();

  // ✅ Step 1: Verify this LoRa message is for this HQ
  if (targetLoraId == LORA_ID) {
    Serial.println("LoRa packet addressed to this HQ.");

    // ✅ Step 2: If TGT matches this HQ_ID OR is "ALL" -> trigger buzzer
    if (tgt == String(HQ_ID) || tgt == "ALL") {
      if (tgt == "ALL") {
        Serial.println("Target is ALL. Activating buzzer for ALL nodes!");
      } else {
        Serial.println("Target matches HQ node ID. Activating buzzer!");
      }
      unsigned long durMs = (dur.length() > 0 ? atol(dur.c_str()) * 1000UL : 5000);
      triggerBuzzerFor(durMs);
      
      // If it's an ALL message, also forward to branch nodes
      if (tgt == "ALL") {
        Serial.println("Forwarding ALL message to branch nodes via ESP-NOW...");
        String out = "SRC:" + String(HQ_ID) + "|TGT:" + tgt + "|DUR:" + dur + "|MSG:" + msg;
        forwardViaEspNow(out);
      }
    }
    // ✅ Step 3: Otherwise, forward to branch nodes
    else {
      Serial.println("Target is a branch node. Forwarding via ESP-NOW...");
      String out = "SRC:" + String(HQ_ID) + "|TGT:" + tgt + "|DUR:" + dur + "|MSG:" + msg;
      forwardViaEspNow(out);
    }
  } else {
    Serial.printf("Not for this HQ (DST=%d). Ignored.\n", targetLoraId);
  }
}

// ---------------- LoRa setup ----------------
void setupLoRa() {
  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  Serial.printf("Init LoRa @ %.0f MHz\n", LORA_FREQ / 1e6);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed! Continuing without LoRa.");
    lora_ok = false;
    return;
  }
  
  LoRa.receive();
  Serial.println("LoRa ready.");
  lora_ok = true;
}

// ---------------- ESP-NOW setup ----------------
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed! Continuing without ESP-NOW.");
    espnow_ok = false;
    return;
  }
  
  espnow_ok = true;

  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  for (int i = 0; i < NUM_PEERS; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peers[i], 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK)
      Serial.printf("Added peer %d to ESP-NOW\n", i + 1);
    else
      Serial.printf("Failed to add peer %d\n", i + 1);
  }
}

// ---------------- forward via ESP-NOW ----------------
void forwardViaEspNow(const String &payload) {
  if (!espnow_ok) {
    Serial.println("ESP-NOW not available, skipping forward");
    return;
  }
  
  Serial.println("Forwarding via ESP-NOW: " + payload);
  for (int i = 0; i < NUM_PEERS; i++) {
    esp_err_t res = esp_now_send(peers[i], (uint8_t *)payload.c_str(), payload.length());
    if (res != ESP_OK)
      Serial.printf("esp_now_send peer %d failed: %d\n", i, res);
  }
}

// ---------------- forward via LoRa ----------------
void forwardViaLoRa(const String &payload) {
  if (!lora_ok) {
    Serial.println("⚠ LoRa not available, skipping forward");
    return;
  }
  
  unsigned long now = millis();
  if (now - lastLoRaSend < SEND_COOLDOWN_MS) {
    Serial.println("LoRa send cooldown, skipping.");
    return;
  }
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  lastLoRaSend = now;
  Serial.println("LoRa TX: " + payload);
  LoRa.receive();
}

// ---------------- trigger buzzer ----------------
void triggerBuzzerFor(unsigned long ms) {
  if (ms == 0) ms = 5000;
  Serial.printf("Buzzing for %.1f sec...\n", ms / 1000.0);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  alertStart = millis();
  alertDurationMs = ms;
  alertActive = true;
}

// ---------------- small beep ----------------
void beep(int t) {
  for (int i = 0; i < t; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// ✅ ---------------- ESP-NOW receive callback (Instant LoRa Forward) ----------------
void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  const uint8_t *mac = info->src_addr;
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  int L = len > 249 ? 249 : len;
  char buf[250];
  memcpy(buf, incomingData, L);
  buf[L] = '\0';
  String msg = String(buf);

  Serial.printf("ESP-NOW RX from %s: %s\n", macStr, msg.c_str());

  // ✅ Immediately forward to LoRa if available
  if (lora_ok) {
    String out = "DST:" + String(LORA_ID) + "|SRC:" + String(HQ_ID) + "|MSG:" + msg;
    Serial.println("Forwarding immediately via LoRa: " + out);
    forwardViaLoRa(out);
  } else {
    Serial.println("LoRa not available, cannot forward message");
  }
}

// ✅ ---------------- ESP-NOW send callback ----------------
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  char macStr[18];
  if (info) {
    const uint8_t *mac_addr = info->des_addr;
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac_addr[0], mac_addr[1], mac_addr[2],
            mac_addr[3], mac_addr[4], mac_addr[5]);
  } else strcpy(macStr, "UNKNOWN");

  Serial.printf("ESP-NOW send to %s -> %s\n",
                macStr,
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ---------------- SENSOR CHECK ----------------
void checkSensors() {
  unsigned long now = millis();
  String faultList = "";

  float lux = -1;
  float tiltAngle = -1;
  String lightStatus = "N/A";

  if (bh1750_ok) {
    lux = lightMeter.readLightLevel();
    if (lux >= 0 && lux < 200000) {
      luxSamples[sampleIndex] = lux;
      lightStates[sampleIndex] = (lux >= LUX_THRESHOLD);
      sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
      if (sampleIndex == 0) bufferFilled = true;

      int validCount = bufferFilled ? SAMPLE_COUNT : sampleIndex;
      int onCount = 0, offCount = 0, stateChanges = 0;
      for (int i = 0; i < validCount; i++) {
        if (lightStates[i]) onCount++;
        else offCount++;
        if (i > 0 && lightStates[i] != lightStates[i - 1]) stateChanges++;
      }

      if (stateChanges >= 3) lightStatus = "FLICKERING";
      else if (onCount == validCount) lightStatus = "ON";
      else if (offCount == validCount) lightStatus = "OFF";
      else lightStatus = "TRANSITION";
    } else {
      faultList += "LIGHT_SENSOR_ERROR, ";
      bh1750_ok = false;
    }
  } else faultList += "LIGHT_SENSOR_NOT_CONNECTED, ";

  if (mpu_ok) {
    int16_t axRaw, ayRaw, azRaw;
    if (mpu.testConnection()) {
      mpu.getAcceleration(&axRaw, &ayRaw, &azRaw);
      float ax = axRaw / 16384.0;
      float ay = ayRaw / 16384.0;
      float az = azRaw / 16384.0;
      float totalG = sqrt(ax * ax + ay * ay + az * az);
      if (totalG > 0.0001) tiltAngle = acos(az / totalG) * (180.0 / PI);
      
      prevAccelX = ax;
      prevAccelY = ay;
      prevAccelZ = az;
      prevAngle = tiltAngle;
    } else {
      faultList += "MPU_SENSOR_ERROR, ";
      mpu_ok = false;
    }
  } else faultList += "MPU_SENSOR_NOT_CONNECTED, ";

  // ACS - Check for current faults
  if (now - lastCurrentRead >= READ_INTERVAL) {
    lastCurrentRead = now;
    float current = getACCurrent();
    
    if (filteredCurrent == 0 || abs(current - filteredCurrent) > 2.0) {
      filteredCurrent = current;
    } else {
      filteredCurrent = alpha * current + (1 - alpha) * filteredCurrent;
    }
    
    if (filteredCurrent < 0) filteredCurrent = 0;
  }

  if (lightStatus == "OFF")           faultList += "LOW_LIGHT, ";
  if (lightStatus == "FLICKERING")    faultList += "FLICKERING, ";
  if (poleFallen) {
    faultList += "POLE_FALL, ";
    poleFallen = false;
  }
  
  // Check for NO_CURRENT fault
  if (filteredCurrent < CURRENT_THRESHOLD) {
    faultList += "NO_CURRENT, ";
  }
  
  if (faultList.endsWith(", ")) faultList.remove(faultList.length() - 2);

  Serial.println("----------- SENSOR STATUS -----------");
  Serial.print("Lux: "); Serial.println(lux);
  Serial.print("Tilt Angle: "); Serial.println(tiltAngle);
  Serial.print("Current: "); Serial.print(filteredCurrent, 3); Serial.println("A");
  Serial.print("Raw Current: "); Serial.print(getACCurrent(), 3); Serial.println("A");
  Serial.print("Sensor Offset: "); Serial.println(sensorOffset);
  Serial.print("MPU Status: "); Serial.println(mpu_ok ? "Connected" : "Disconnected");
  Serial.println("------------------------------------");

  if (faultList.length() > 0) {
    if (now - lastFaultGeneration >= FAULT_GENERATION_COOLDOWN) {
      lastFaultGeneration = now;
      String msg = String("HQ:") + String(HQ_ID) + " -> FAULTS: [" + faultList + "]";
      Serial.println("Generating new fault message: " + msg);
      
      // Only send via available communication methods
      if (espnow_ok) forwardViaEspNow(msg);
      if (lora_ok) {
        String loraPacket = "DST:" + String(LORA_ID) + "|SRC:" + String(HQ_ID) + "|MSG:" + msg;
        forwardViaLoRa(loraPacket);
      }
    } else {
      Serial.println("Fault detected but still in cooldown period...");
    }
  }
}

// ---------------- MPU INITIALIZATION ----------------
bool initializeMPU() {
  for (int attempt = 0; attempt < 3; attempt++) {
    mpu.initialize();
    delay(100);
    
    if (mpu.testConnection()) {
      Serial.println("MPU6050 OK");
      mpu.setSleepEnabled(false);
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
      return true;
    }
    delay(500);
  }
  
  Serial.println("MPU6050 not detected after 3 attempts");
  return false;
}

// ---------------- ASYNCHRONOUS POLE FALL DETECTION ----------------
void checkPoleFallAsync() {
  unsigned long now = millis();
  
  if (now - lastPoleFallCheck < POLE_FALL_CHECK_INTERVAL) {
    return;
  }
  lastPoleFallCheck = now;
  
  int16_t axRaw, ayRaw, azRaw;
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 disconnected during pole fall check");
    mpu_ok = false;
    return;
  }
  
  mpu.getAcceleration(&axRaw, &ayRaw, &azRaw);
  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;
  
  // Simple noise filtering - only process if change is significant
  float changeX = fabs(ax - stableAx);
  float changeY = fabs(ay - stableAy);
  float changeZ = fabs(az - stableAz);
  
  if (changeX < NOISE_THRESHOLD && changeY < NOISE_THRESHOLD && changeZ < NOISE_THRESHOLD) {
    stableCount++;
    if (stableCount >= STABLE_READINGS) {
      // Update stable values
      stableAx = ax;
      stableAy = ay;
      stableAz = az;
    }
  } else {
    // Significant change detected, reset stable count
    stableCount = 0;
  }
  
  // Use stable values for calculations
  float processAx = (stableCount >= STABLE_READINGS) ? stableAx : ax;
  float processAy = (stableCount >= STABLE_READINGS) ? stableAy : ay;
  float processAz = (stableCount >= STABLE_READINGS) ? stableAz : az;
  
  float totalG = sqrt(processAx * processAx + processAy * processAy + processAz * processAz);
  float currentAngle = 0;
  if (totalG > 0.0001) {
    currentAngle = acos(processAz / totalG) * (180.0 / PI);
  }
  
  float accelChange = sqrt(
    pow(processAx - prevAccelX, 2) + 
    pow(processAy - prevAccelY, 2) + 
    pow(processAz - prevAccelZ, 2)
  );
  
  float angleChange = fabs(currentAngle - prevAngle);
  
  // Debug output
  static unsigned long lastDebug = 0;
  // if (now - lastDebug > 200) {
  //   lastDebug = now;
  //   Serial.print("MPU - AccelChange: ");
  //   Serial.print(accelChange, 3);
  //   Serial.print(", AngleChange: ");
  //   Serial.print(angleChange, 2);
  //   Serial.print(", CurrentAngle: ");
  //   Serial.println(currentAngle, 2);
  // }
  
  // Update previous values
  prevAccelX = processAx;
  prevAccelY = processAy;
  prevAccelZ = processAz;
  prevAngle = currentAngle;
  
  // Check cooldown
  if (now - lastPoleFallAlert < POLE_FALL_COOLDOWN) {
    return;
  }
  
  // IMMEDIATE DETECTION - Check any of the three conditions
  bool poleFallDetected = false;
  String detectionReason = "";
  
  // Condition 1: Sudden acceleration change in any axis
  if (accelChange > SPEED_CHANGE_THRESHOLD) {
    poleFallDetected = true;
    detectionReason = "Acceleration change: " + String(accelChange, 3);
  }
  
  // Condition 2: Sudden angle change
  if (angleChange > ANGLE_CHANGE_THRESHOLD) {
    poleFallDetected = true;
    detectionReason = "Angle change: " + String(angleChange, 2) + "°";
  }
  
  // Condition 3: Absolute tilt beyond threshold
  if (currentAngle > ABSOLUTE_TILT_THRESHOLD) {
    poleFallDetected = true;
    detectionReason = "Absolute tilt: " + String(currentAngle, 2) + "°";
  }
  
  // IMMEDIATE ACTION ON FIRST DETECTION
  if (poleFallDetected) {
    poleFallen = true;
    lastPoleFallAlert = now; // Reset the 30s timer immediately
    
    Serial.println("POLE FALL DETECTED! Reason: " + detectionReason);
    Serial.println("Sending immediate fault via LoRa...");
    
    sendPoleFallFault();
    // beep(5);
    
    // Reset flag after sending
    poleFallen = false;
  }
}

// ---------------- SEND POLE FALL FAULT ----------------
void sendPoleFallFault() {
  String msg = String("HQ:") + String(HQ_ID) + " -> FAULTS: [POLE_FALL]";
  Serial.println("Sending immediate pole fall fault via LoRa: " + msg);
  
  // MODIFIED: Only send via LoRa for pole fall alerts
  if (lora_ok) {
    String loraPacket = "DST:" + String(LORA_ID) + "|SRC:" + String(HQ_ID) + "|MSG:" + msg;
    forwardViaLoRa(loraPacket);
  } else {
    Serial.println("LoRa not available, cannot send pole fall alert");
  }
}

// ---------------- MPU Calibration ----------------
void calibrateMPU() {
  Serial.println("Calibrating MPU6050... Keep stable for 3s.");
  delay(3000);
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 50; i++) {
    if (!mpu.testConnection()) { mpu_ok = false; return; }
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    sumX += ax / 16384.0;
    sumY += ay / 16384.0;
    sumZ += az / 16384.0;
    delay(50);
  }
  baselineX = sumX / 50.0;
  baselineY = sumY / 50.0;
  baselineZ = sumZ / 50.0;
  
  prevAccelX = baselineX;
  prevAccelY = baselineY;
  prevAccelZ = baselineZ;
  prevAngle = 0;
  
  calibrated = true;
  Serial.println("MPU calibrated.");
  Serial.printf("Baseline X=%.2f Y=%.2f Z=%.2f\n", baselineX, baselineY, baselineZ);
}

// ---------------- UPDATED: ACS712 Calibration ----------------
float getSensorOffset() {
  // First, wait 60 seconds for sensor stabilization
  Serial.println("Waiting 60 seconds for sensor stabilization...");
  digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate stabilization period
  
  unsigned long stabilizationStart = millis();
  while (millis() - stabilizationStart < 11000) {
    // Blink LED every 5 seconds to show progress
    if ((millis() - stabilizationStart) % 5000 < 100) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
    
    // Print progress every 10 seconds
    if ((millis() - stabilizationStart) % 10000 < 100) {
      int secondsRemaining = (11000 - (millis() - stabilizationStart)) / 1000;
      Serial.print("Stabilizing... ");
      Serial.print(secondsRemaining);
      Serial.println(" seconds remaining");
    }
    
    delay(100);
  }
  
  digitalWrite(LED_PIN, LOW);  // Turn off LED
  Serial.println("Stabilization complete. Now taking 10 readings for calibration...");
  
  // Now take 10 readings and average them
  float readings[10];
  float sum = 0;
  
  digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate calibration period
  
  for (int i = 0; i < 10; i++) {
    readings[i] = analogRead(ACS_PIN);
    sum += readings[i];
    
    Serial.print("Reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(readings[i]);
    
    // Brief LED blink for each reading
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(800);
  }
  
  digitalWrite(LED_PIN, LOW);  // Turn off LED
  
  float average = sum / 10.0;
  Serial.print("Calibration complete. Average of 10 readings: ");
  Serial.println(average);
  
  return average;
}

// ---------------- ACS712 Current Measurement ----------------
float getACCurrent() {
  const int samples = 200;
  long sum = 0;
  int sampleCount = 0;
  
  unsigned long startTime = micros();
  
  while (micros() - startTime < 20000 && sampleCount < samples) {
    int val = analogRead(ACS_PIN);
    float centered = val - sensorOffset;
    sum += centered * centered;
    sampleCount++;
    delayMicroseconds(50);
  }
  
  if (sampleCount == 0) return 0;
  
  float rms = sqrt(sum / (float)sampleCount);
  float voltage = rms * (3.3 / 4095.0);
  float sensitivity = 0.185;
  float current = voltage / sensitivity;
  current *= 0.9;
  
  if (current < 0) current = 0;
  
  return current;
}

// ---------------- Current Sensor Recalibration ----------------
void recalibrateCurrentSensor() {
  const int quickSamples = 100;
  long sum = 0;
  
  for (int i = 0; i < quickSamples; i++) {
    sum += analogRead(ACS_PIN);
    delay(1);
  }
  
  float newOffset = sum / (float)quickSamples;
  
  if (abs(newOffset - sensorOffset) > 2.0) {
    sensorOffset = newOffset;
    Serial.print("Recalibrated sensor offset: ");
    Serial.println(sensorOffset);
  }
  
  lastRecalibration = millis();
}
