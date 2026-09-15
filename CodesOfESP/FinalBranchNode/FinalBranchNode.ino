#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <BH1750.h>
#include <MPU6050.h>
#include <math.h>

// ---------------- CONFIG ----------------
#define NODE_ID "4001"     // change this for each branch node
#define BUZZER_PIN 12      // D6
#define ACS_PIN A0
#define LED_PIN 0          // D3
#define WIFI_CHANNEL 1

// ---------------- ESP-NOW PEERS ----------------
uint8_t prevNode[] = {0xF8,0xB3,0xB7,0x87,0x33,0xAA};  // Previous node MAC
uint8_t nextNode[] = {0x6C,0xC8,0x40,0x4F,0x69,0x94};  // Next node MAC

// ---------------- OBJECTS ----------------
BH1750 lightMeter;
MPU6050 mpu;

// ---------------- FLAGS ----------------
bool bh1750_ok = false;
bool mpu_ok = false;


// ---------------- THRESHOLDS / PARAMETERS ----------------
const float LUX_THRESHOLD = 300.0;
const unsigned long READ_INTERVAL = 1000;
const float TILT_THRESHOLD = 45.0;
const float CURRENT_THRESHOLD = 0.035;
const float alpha = 0.2;
const int SAMPLE_COUNT = 10;

// Pole fallen detection parameters - BALANCED SENSITIVITY
const float SPEED_CHANGE_THRESHOLD = 0.8;  // Sensitive but not too sensitive
const float ANGLE_CHANGE_THRESHOLD = 10.0; // Sensitive but not too sensitive
const unsigned long POLE_FALL_COOLDOWN = 30000;
const unsigned long POLE_FALL_CHECK_INTERVAL = 50; // Check every 50ms

// NEW: Direction-specific acceleration thresholds
const float X_ACCEL_THRESHOLD = 1.5;  // Threshold for X-axis acceleration (g)
const float Y_ACCEL_THRESHOLD = 1.5;  // Threshold for Y-axis acceleration (g)
const float Z_ACCEL_THRESHOLD = 2.0;  // Threshold for Z-axis acceleration (g)
const float ACCEL_DURATION_THRESHOLD = 100; // Minimum duration (ms) for acceleration to be considered

// Noise filtering
const float NOISE_THRESHOLD = 0.1; // Minimum change to consider
const int STABLE_READINGS = 2; // Need 2 stable readings before processing

// ---------------- GLOBAL VARS ----------------
float sensorOffset = 0.0;
float filteredCurrent = 0.0;
bool calibrated = false;
bool poleFallen = false;
float baselineX = 0.0, baselineY = 0.0, baselineZ = 1.0;

// For tracking changes in acceleration and angle
float prevAccelX = 0.0, prevAccelY = 0.0, prevAccelZ = 0.0;
float prevAngle = 0.0;
unsigned long lastPoleFallAlert = 0;
unsigned long lastPoleFallCheck = 0;
unsigned long lastMPUCheck = 0;
const unsigned long MPU_RECHECK_INTERVAL = 5000;

// NEW: Variables for direction-specific acceleration detection
unsigned long xAccelStartTime = 0;
unsigned long yAccelStartTime = 0;
unsigned long zAccelStartTime = 0;
bool xAccelDetected = false;
bool yAccelDetected = false;
bool zAccelDetected = false;

// Noise filtering variables
float stableAx = 0.0, stableAy = 0.0, stableAz = 0.0;
int stableCount = 0;

// Current sensor specific variables
unsigned long lastRecalibration = 0;
const unsigned long RECALIBRATION_INTERVAL = 11000;

// Fault generation cooldown
static unsigned long lastFaultGeneration = 0;
const unsigned long FAULT_GENERATION_COOLDOWN = 10000;

float luxSamples[SAMPLE_COUNT];
bool lightStates[SAMPLE_COUNT];
int sampleIndex = 0;
bool bufferFilled = false;

unsigned long lastCurrentRead = 0;
unsigned long lastSend = 0;

volatile bool msgPending = false;
String pendingMsg = "";
uint8_t pendingSender[6];

// ---------------- ALERT BUZZER VARIABLES ----------------
unsigned long alertStart = 0;
bool alertActive = false;
unsigned long alertDurationMs = 0;

// ---------------- FUNCTION DECLARATIONS ----------------
void sendTo(const uint8_t *mac, const String &msg);
void forwardMsg(const String &msg, const uint8_t *exclude);
void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len);
void beep(int t);
float getSensorOffset();
float getACCurrent();
void calibrateMPU();
void checkSensors();
void triggerBuzzerFor(unsigned long ms);
void processIncomingMessage(const String &msg, const uint8_t *sender);
bool isSender(const uint8_t *mac1, const uint8_t *mac2);
void printMacAddress(const uint8_t *mac);
void recalibrateCurrentSensor();
void checkPoleFallAsync();
bool initializeMPU();
void sendPoleFallFault();

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Serial.println("=== NODE " + String(NODE_ID) + " STARTING ===");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(D2, D1);
  Wire.setClock(100000);

  // Initialize MPU6050 first
  mpu_ok = initializeMPU();
  
  // Sensors
  bh1750_ok = lightMeter.begin();
  if (bh1750_ok) Serial.println("BH1750 OK");
  else Serial.println("BH1750 not detected");

  // Calibrate current sensor - wait 60s for stabilization, then average 10 readings
  Serial.println("Calibrating current sensor... This will take approximately 70 seconds.");
  sensorOffset = getSensorOffset();
  Serial.print("Initial Sensor Offset: "); Serial.println(sensorOffset);
  lastRecalibration = millis();
  
  if (mpu_ok) calibrateMPU();

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);

  // Add peers with error checking
  if (esp_now_add_peer(prevNode, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0) != 0) {
    Serial.println("Failed to add previous node as peer");
  } else {
    Serial.print("Added previous node as peer: ");
    printMacAddress(prevNode);
  }
  
  if (esp_now_add_peer(nextNode, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0) != 0) {
    Serial.println("Failed to add next node as peer");
  } else {
    Serial.print("Added next node as peer: ");
    printMacAddress(nextNode);
  }

  Serial.println("System Ready, Node: " NODE_ID);
}

// ---------------- LOOP ----------------
void loop() {
  static unsigned long lastCheck = 0;

  if (msgPending) {
    processIncomingMessage(pendingMsg, pendingSender);
    msgPending = false;
  }

  if (alertActive && (millis() - alertStart >= alertDurationMs)) {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    alertActive = false;
    Serial.println("Alert completed, buzzer off.");
  }

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
  
  // NEW: Check for direction-specific acceleration
  bool xAccelTriggered = false;
  bool yAccelTriggered = false;
  bool zAccelTriggered = false;
  
  // X-axis acceleration check
  if (fabs(processAx) > X_ACCEL_THRESHOLD) {
    if (!xAccelDetected) {
      xAccelDetected = true;
      xAccelStartTime = now;
    } else if (now - xAccelStartTime > ACCEL_DURATION_THRESHOLD) {
      xAccelTriggered = true;
    }
  } else {
    xAccelDetected = false;
  }
  
  // Y-axis acceleration check
  if (fabs(processAy) > Y_ACCEL_THRESHOLD) {
    if (!yAccelDetected) {
      yAccelDetected = true;
      yAccelStartTime = now;
    } else if (now - yAccelStartTime > ACCEL_DURATION_THRESHOLD) {
      yAccelTriggered = true;
    }
  } else {
    yAccelDetected = false;
  }
  
  // Z-axis acceleration check
  if (fabs(processAz - 1.0) > Z_ACCEL_THRESHOLD) {  // Subtract 1.0 to account for gravity
    if (!zAccelDetected) {
      zAccelDetected = true;
      zAccelStartTime = now;
    } else if (now - zAccelStartTime > ACCEL_DURATION_THRESHOLD) {
      zAccelTriggered = true;
    }
  } else {
    zAccelDetected = false;
  }
  
  // Debug output
  static unsigned long lastDebug = 0;
  // if (now - lastDebug > 200) {
  //   lastDebug = now;
  //   Serial.print("🔍 MPU - Accel: X=");
  //   Serial.print(processAx, 3);
  //   Serial.print(", Y=");
  //   Serial.print(processAy, 3);
  //   Serial.print(", Z=");
  //   Serial.print(processAz, 3);
  //   Serial.print(", Total=");
  //   Serial.print(totalG, 3);
  //   Serial.print(", Angle=");
  //   Serial.print(currentAngle, 2);
  //   Serial.print("°, AccelChange=");
  //   Serial.print(accelChange, 3);
  //   Serial.print(", AngleChange=");
  //   Serial.println(angleChange, 2);
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
  
  // IMMEDIATE DETECTION - Check any of the conditions
  bool poleFallDetected = false;
  String detectionReason = "";
  
  // Condition 1: Sudden acceleration change
  if (accelChange > SPEED_CHANGE_THRESHOLD) {
    poleFallDetected = true;
    detectionReason = "Acceleration change: " + String(accelChange, 3);
  }
  
  // Condition 2: Sudden angle change
  if (angleChange > ANGLE_CHANGE_THRESHOLD) {
    poleFallDetected = true;
    detectionReason = "Angle change: " + String(angleChange, 2) + "°";
  }
  
  // NEW: Condition 3: X-axis acceleration
  if (xAccelTriggered) {
    poleFallDetected = true;
    detectionReason = "X-axis acceleration: " + String(processAx, 3) + "g";
  }
  
  // NEW: Condition 4: Y-axis acceleration
  if (yAccelTriggered) {
    poleFallDetected = true;
    detectionReason = "Y-axis acceleration: " + String(processAy, 3) + "g";
  }
  
  // NEW: Condition 5: Z-axis acceleration
  if (zAccelTriggered) {
    poleFallDetected = true;
    detectionReason = "Z-axis acceleration: " + String(fabs(processAz - 1.0), 3) + "g";
  }
  
  // IMMEDIATE ACTION ON FIRST DETECTION
  if (poleFallDetected) {
    poleFallen = true;
    lastPoleFallAlert = now;
    
    Serial.println("POLE FALL DETECTED! Reason: " + detectionReason);
    Serial.println("Sending immediate fault...");
    
    sendPoleFallFault();
    // beep(5);
    
    // Reset flags after sending
    poleFallen = false;
    xAccelDetected = false;
    yAccelDetected = false;
    zAccelDetected = false;
  }
}

// ---------------- SEND POLE FALL FAULT ----------------
void sendPoleFallFault() {
  String msg = String(NODE_ID) + " -> FAULTS: [POLE_FALL]";
  Serial.println("Sending immediate pole fall fault: " + msg);
  forwardMsg(msg, NULL);
}

// ---------------- PROCESS INCOMING ----------------
void processIncomingMessage(const String &msg, const uint8_t *sender) {
  Serial.print("Received from ");
  printMacAddress(sender);
  Serial.println(": " + msg);

  String src = "", tgt = "", dur = "", cmd = "";
  bool isAlertFormat = false;
  
  if (msg.indexOf("SRC:") >= 0) {
    isAlertFormat = true;
    int pSrc = msg.indexOf("SRC:");
    int pTgt = msg.indexOf("|TGT:");
    int pDur = msg.indexOf("|DUR:");
    int pMsg = msg.indexOf("|MSG:");

    if (pSrc >= 0 && pTgt > pSrc) src = msg.substring(pSrc + 4, pTgt);
    if (pTgt >= 0 && pDur > pTgt) tgt = msg.substring(pTgt + 5, pDur);
    if (pDur >= 0 && pMsg > pDur) dur = msg.substring(pDur + 5, pMsg);
    if (pMsg >= 0) cmd = msg.substring(pMsg + 5);

    src.trim(); tgt.trim(); dur.trim(); cmd.trim();
    
    Serial.println("Parsed: SRC=" + src + ", TGT=" + tgt + ", DUR=" + dur + ", MSG=" + cmd);
  }

  if (isAlertFormat && (tgt == NODE_ID || tgt == "ALL")) {
    Serial.println("Target matches this node ID or ALL → Activating buzzer.");
    if (cmd.equalsIgnoreCase("ALERT")) {
      unsigned long durMs = dur.toInt() * 1000UL;
      if (durMs == 0) durMs = 5000;
      triggerBuzzerFor(durMs);
    }
  }
  else if (String(NODE_ID) == "4001" && isAlertFormat && cmd.equalsIgnoreCase("ALERT")) {
    Serial.println("ALERT not for 4001 → forwarding backward to previous node.");
    Serial.print("Forwarding to previous node: ");
    printMacAddress(prevNode);
    sendTo(prevNode, msg);
  }
  else {
    Serial.println("↩ Normal message flow → forward to both directions if applicable.");
    forwardMsg(msg, sender);
  }
  
  if (isAlertFormat && tgt == "ALL") {
    Serial.println("Forwarding ALL message to ensure it reaches all nodes");
    forwardMsg(msg, sender);
  }
}

// ---------------- TRIGGER BUZZER ----------------
void triggerBuzzerFor(unsigned long ms) {
  Serial.printf("Buzzing for %.1f sec...\n", ms / 1000.0);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  alertStart = millis();
  alertDurationMs = ms;
  alertActive = true;
}

// ---------------- SENSOR CHECK ----------------
void checkSensors() {
  unsigned long now = millis();
  String faultList = "";

  float lux = -1, tiltAngle = -1;
  String lightStatus = "N/A";

  // BH1750
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

  // MPU6050
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
      
      if (!poleFallen && calibrated) {
        float dx = fabs(ax - baselineX);
        float dy = fabs(ay - baselineY);
        float dz = fabs(az - baselineZ);
        if (dx > 0.3 || dy > 0.3 || dz > 0.4 || tiltAngle > TILT_THRESHOLD)
          poleFallen = true;
      }
    } else {
      faultList += "MPU_SENSOR_ERROR, ";
      mpu_ok = false;
    }
  } else faultList += "MPU_SENSOR_NOT_CONNECTED, ";

  // ACS - Always check for NO_CURRENT fault
  bool checkCurrentFault = false;
  if (now - lastCurrentRead >= READ_INTERVAL) {
    lastCurrentRead = now;
    float current = getACCurrent();
    
    if (filteredCurrent == 0 || abs(current - filteredCurrent) > 2.0) {
      filteredCurrent = current;
    } else {
      filteredCurrent = alpha * current + (1 - alpha) * filteredCurrent;
    }
    
    if (filteredCurrent < 0) filteredCurrent = 0;
    
    // Always check for current fault
    checkCurrentFault = true;
  }

  // Faults
  if (lightStatus == "OFF") faultList += "LOW_LIGHT, ";
  if (lightStatus == "FLICKERING") faultList += "FLICKERING, ";
  if (poleFallen) {
    faultList += "TILT, ";
    poleFallen = false;
  }
  
  // Always add NO_CURRENT fault if detected
  if (checkCurrentFault && filteredCurrent < CURRENT_THRESHOLD) {
    faultList += "NO_CURRENT, ";
  }
  
  if (faultList.endsWith(", ")) faultList.remove(faultList.length() - 2);

  Serial.println("----------- SENSOR STATUS -----------");
  Serial.print("Lux: "); Serial.println(lux);
  Serial.print("Tilt: "); Serial.println(tiltAngle);
  Serial.print("Current: "); Serial.print(filteredCurrent, 3); Serial.println("A");
  Serial.print("Raw Current: "); Serial.print(getACCurrent(), 3); Serial.println("A");
  Serial.print("Sensor Offset: "); Serial.println(sensorOffset);
  Serial.print("MPU Status: "); Serial.println(mpu_ok ? "Connected" : "Disconnected");
  Serial.println("------------------------------------");

  if (faultList.length() > 0) {
    if (now - lastFaultGeneration >= FAULT_GENERATION_COOLDOWN) {
      lastFaultGeneration = now;
      String msg = String(NODE_ID) + " -> FAULTS: [" + faultList + "]";
      Serial.println("Generating new fault message: " + msg);
      forwardMsg(msg, NULL);
    } else {
      Serial.println("Fault detected but still in cooldown period...");
    }
  }
}

// ---------------- UTILITIES ----------------
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
  float voltage = rms * (3.3 / 1023.0);
  float sensitivity = 0.185;
  float current = voltage / sensitivity;
  current *= 0.9;
  
  if (current < 0) current = 0;
  
  return current;
}

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

void calibrateMPU() {
  Serial.println("Calibrating MPU6050... Keep stable 3s.");
  delay(3000);
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 50; i++) {
    if (!mpu.testConnection()) {
      mpu_ok = false;
      Serial.println("MPU disconnected during calibration — skipping.");
      return;
    }
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    sx += ax / 16384.0;
    sy += ay / 16384.0;
    sz += az / 16384.0;
    delay(50);
  }
  baselineX = sx / 50.0;
  baselineY = sy / 50.0;
  baselineZ = sz / 50.0;
  
  prevAccelX = baselineX;
  prevAccelY = baselineY;
  prevAccelZ = baselineZ;
  prevAngle = 0;
  
  calibrated = true;
  Serial.println("MPU Calibration done");
}

void beep(int t) {
  for (int i = 0; i < t; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// ---------------- ESP-NOW CORE ----------------
void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  char buffer[250];
  if (len > 249) len = 249;
  memcpy(buffer, incomingData, len);
  buffer[len] = '\0';
  pendingMsg = String(buffer);
  memcpy(pendingSender, mac, 6);
  msgPending = true;
}

void sendTo(const uint8_t *mac, const String &msg) {
  uint8_t result = esp_now_send((uint8_t *)mac, (uint8_t *)msg.c_str(), msg.length());
  
  if (result == 0) {
    Serial.printf("Sent to %02X:%02X:%02X:%02X:%02X:%02X | %s\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], msg.c_str());
  } else {
    Serial.printf("Failed to send to %02X:%02X:%02X:%02X:%02X:%02X | Error: %d\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], result);
  }
}

void forwardMsg(const String &msg, const uint8_t *exclude) {
  bool isPrevSender = (exclude != NULL && isSender(exclude, prevNode));
  bool isNextSender = (exclude != NULL && isSender(exclude, nextNode));
  
  if (!isPrevSender) {
    Serial.print("Forwarding to previous node: ");
    printMacAddress(prevNode);
    sendTo(prevNode, msg);
  }
  
  if (!isNextSender) {
    Serial.print("Forwarding to next node: ");
    printMacAddress(nextNode);
    sendTo(nextNode, msg);
  }
}

bool isSender(const uint8_t *mac1, const uint8_t *mac2) {
  for (int i = 0; i < 6; i++) {
    if (mac1[i] != mac2[i]) return false;
  }
  return true;
}

void printMacAddress(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}
