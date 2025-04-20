/******************************************************************************
 * CubeSat Reaction Wheel — Secondary (Command & Control) Board Code
 * [Integrated Version with Bluetooth + UART Protocol + Telemetry Support]
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "ArduinoBLE.h"
#include <Arduino_LSM6DS3.h>

// ---------------------------------------------------------------------------
// 1. Pin Definitions and Constants
// ---------------------------------------------------------------------------
const int LED_PIN = LED_BUILTIN;
const unsigned long TELEMETRY_INTERVAL_MS = 100;
const float SAMPLE_TIME_S = 0.10f;
const unsigned long LED_BLINK_INTERVAL_MS = 50;
bool newBLEcommand = false;
String incoming;
float ax, ay, az, gx, gy, gz;
float yaw = 0.0f;
unsigned long lastYawUpdateTime = 0;
bool initAttitudeControl = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TELEMETRY_CHAR_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a9"  // New telemetry characteristic

#define CONTROL_ENABLE true
#define TARGET_YAW 0.0f
#define MAX_CONTROL_TORQUE 3000.0f

const float ATTITUDE_KP = 300.0f;
const float ATTITUDE_KI = 0.0f;
const float ATTITUDE_KD = 0.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;

// BLE Components
BLEDevice peripheral;
BLECharacteristic targetChar;
BLECharacteristic telemetryChar;  // New characteristic for telemetry
bool telemetryCharAvailable = false;  // Flag to check if telemetry char is available

// ---------------------------------------------------------------------------
// UART Protocol Definitions
// ---------------------------------------------------------------------------
#define PACKET_START_BYTE    0xAA
#define PACKET_END_BYTE      0x55
#define CMD_TYPE_SPEED       0x01
#define CMD_TYPE_TORQUE      0x02
#define CMD_TYPE_STATUS_REQ  0x03

// ---------------------------------------------------------------------------
// 2. Global Variables
// ---------------------------------------------------------------------------
struct Orientation {
  float roll;
  float pitch;
  float yaw;
};

Orientation currentOrientation = {0.0f, 0.0f, 0.0f};
Orientation targetOrientation = {0.0f, 0.0f, TARGET_YAW};

unsigned long lastTelemetryMillis = 0;
unsigned long lastControlMillis = 0;
unsigned long lastLedMillis = 0;
unsigned long previousMicros = 0;

bool ledState = false;
float gyroRates[3] = {0.0f, 0.0f, 0.0f};
float previousRoll = 0.0f;
float previousPitch = 0.0f;

float yawError = 0.0f;
float yawErrorIntegral = 0.0f;
float yawErrorPrevious = 0.0f;
float commandedTorque = 0.0f;

bool manualCommandActive = true;
float manualCommandTorque = 0.0f;
float manualCommandRPM = 0.0f;

// ---------------------------------------------------------------------------
// Primary Board Integration
// ---------------------------------------------------------------------------
#define PRIMARY_BAUD 115200
HardwareSerial &primarySerial = Serial1;

int16_t currentWheelRPM = 0;
float currentWheelTorque = 0.0f;
uint8_t primaryStatus = 0;
uint8_t primaryFaults = 0;

// ---------------------------------------------------------------------------
// 3. Forward Declarations
// ---------------------------------------------------------------------------
void initIMU();
void readIMU();
void calculateOrientation(float dt);
void updateAttitudeControl();
void handleSerialCommands();
void sendLocalTelemetry();
void sendBinaryCommand(uint8_t cmdType, int16_t value);
void processPrimaryPacket();
void updateLedBlink();
void sendBLETelemetry();  // New function to send telemetry over BLE

// ---------------------------------------------------------------------------
// 4. setup()
// ---------------------------------------------------------------------------
void setup() {
  delay(5000);
  Serial.begin(115200);
  delay(500);
  pinMode(7, OUTPUT);
  Serial.println("=== Reaction Wheel C&C Board (Integrated with Telemetry) ===");
  
  // BLE Initialization
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  Serial.println("BLE initialized");
  BLE.scan();
  Serial.println("Scanning for BLE devices");
  
  // UART Initialization
  primarySerial.begin(PRIMARY_BAUD);
  
  // IMU Initialization
  Wire.begin();
  Serial.println("Initializing IMU");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  Serial.println("IMU initialized");
  targetOrientation.yaw = TARGET_YAW;
  previousMicros = micros();
  
  pinMode(LED_PIN, OUTPUT);
  Serial.println("System Initialized");
}

// ---------------------------------------------------------------------------
// 5. loop()
// ---------------------------------------------------------------------------
void loop() {
  // BLE Handling
  if (!peripheral) {
    BLEDevice discoveredDevice = BLE.available();
    if (discoveredDevice) {
      Serial.print("Found device: ");
      Serial.println(discoveredDevice.address());

      if (discoveredDevice.advertisedServiceUuid() == SERVICE_UUID) {
        Serial.println("Target peripheral found. Connecting...");
        BLE.stopScan();

        if (discoveredDevice.connect()) {
          Serial.println("Connected to peripheral");
          if (discoveredDevice.discoverAttributes()) {
            // Get command characteristic
            targetChar = discoveredDevice.characteristic(CHARACTERISTIC_UUID);
            if (targetChar && targetChar.canSubscribe()) {
              if (targetChar.subscribe()) {
                Serial.println("Subscribed to command notifications");
                digitalWrite(7, HIGH);
                peripheral = discoveredDevice;
              }
            }
            
            // Try to get telemetry characteristic
            telemetryChar = discoveredDevice.characteristic(TELEMETRY_CHAR_UUID);
            if (telemetryChar) {
              telemetryCharAvailable = true;
              Serial.println("Found telemetry characteristic");
            } else {
              telemetryCharAvailable = false;
              Serial.println("Telemetry characteristic not found");
            }
          }
        }
      }
    }
  }

  // Process BLE Notifications
  if (peripheral && peripheral.connected() && targetChar && targetChar.valueUpdated()) {
    int len = targetChar.valueLength();
    const uint8_t* data = targetChar.value();
    incoming = "";
    newBLEcommand = true;
    for (int i = 0; i < len; i++) {
      incoming += (char)data[i];
    }
  }

  // Handle BLE Disconnections
  if (peripheral && !peripheral.connected()) {
    Serial.println("Peripheral disconnected. Restarting scan...");
    digitalWrite(7, LOW);
    peripheral = BLEDevice();
    telemetryCharAvailable = false;
    BLE.scan();
  }

  // Process New BLE Commands
  if (newBLEcommand) {
    Serial.print("New BLE Command: ");
    Serial.println(incoming);
    newBLEcommand = false;
    handleSerialCommands();
  }
  
  readIMU();
  
  float dt = (micros() - previousMicros) / 1000000.0f;
  previousMicros = micros();
  
  if (millis() - lastControlMillis >= (SAMPLE_TIME_S * 1000)) {
    lastControlMillis = millis();
    if(!manualCommandActive){
      updateAttitudeControl();
    }
  }
  
  if (primarySerial.available() >= 9) {
    processPrimaryPacket();
  }
  
  updateLedBlink();
  
  if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMillis = millis();
    sendLocalTelemetry();
    sendBinaryCommand(CMD_TYPE_STATUS_REQ,0);
    
    // Send telemetry over BLE if connection is available
    if (peripheral && peripheral.connected() && telemetryCharAvailable) {
      sendBLETelemetry();
    }
  }
}

// ---------------------------------------------------------------------------
// 6. IMU Functions
// ---------------------------------------------------------------------------
void readIMU() {
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // Convert gyroscope raw data to degrees/second
  gyroRates[0] = gx / 131.0f;  // X
  gyroRates[1] = gy / 131.0f;  // Y
  gyroRates[2] = gz / 131.0f;  // Z

  // Estimate roll and pitch from accelerometer
  float accelRoll  = atan2(ay, az) * 57.2958f;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;

  // Update roll and pitch
  currentOrientation.roll  = accelRoll;
  currentOrientation.pitch = accelPitch;

  // Yaw integration
  unsigned long currentTime = millis();
  float dt = (currentTime - lastYawUpdateTime) / 1000.0f; // convert ms to seconds
  lastYawUpdateTime = currentTime;

  float gz_dps = gz / 131.0f;  // Convert gz to deg/sec
  yaw += gz_dps * dt*150;

  // Normalize yaw to 0–360
  if (yaw >= 360.0f) yaw -= 360.0f;
  if (yaw < 0.0f)   yaw += 360.0f;

  currentOrientation.yaw = gz_dps;
}

// ---------------------------------------------------------------------------
// 7. Control & Communication (Modified Section)
// ---------------------------------------------------------------------------
void updateAttitudeControl() {
  // Serial.println("DEBUG: [updateAttitudeControl] Entry");
  // Serial.print("DEBUG: [updateAttitudeControl] CONTROL_ENABLE=");
  // Serial.println(CONTROL_ENABLE);
  
  if (!CONTROL_ENABLE) {
    Serial.println("DEBUG: [updateAttitudeControl] Control disabled, exiting function");
    return;
  }
  
  // Serial.print("DEBUG: [updateAttitudeControl] Target yaw=");
  // Serial.print(targetOrientation.yaw);
  // Serial.print(", Current yaw=");
  // Serial.println(currentOrientation.yaw);
  
  yawError = targetOrientation.yaw - currentOrientation.yaw;
  // Serial.print("DEBUG: [updateAttitudeControl] Calculated yawError=");
  // Serial.println(yawError);
  
  // Serial.print("DEBUG: [updateAttitudeControl] Before integration, yawErrorIntegral=");
  // Serial.println(yawErrorIntegral);
  
  yawErrorIntegral += yawError * SAMPLE_TIME_S;
  
  // Serial.print("DEBUG: [updateAttitudeControl] After integration (before constraint), yawErrorIntegral=");
  // Serial.println(yawErrorIntegral);
  
  yawErrorIntegral = constrain(yawErrorIntegral, -100.0f, 100.0f);
  
  // Serial.print("DEBUG: [updateAttitudeControl] After constraint, yawErrorIntegral=");
  // Serial.println(yawErrorIntegral);
  
  // Serial.print("DEBUG: [updateAttitudeControl] SAMPLE_TIME_S=");
  // Serial.print(SAMPLE_TIME_S);
  // Serial.print(", Previous yaw error=");
  // Serial.println(yawErrorPrevious);
  
  float yawErrorDerivative = (yawError - yawErrorPrevious) / SAMPLE_TIME_S;
  
  // Serial.print("DEBUG: [updateAttitudeControl] Calculated yawErrorDerivative=");
  // Serial.println(yawErrorDerivative);
  
  yawErrorPrevious = yawError;
  // Serial.print("DEBUG: [updateAttitudeControl] Updated yawErrorPrevious=");
  // Serial.println(yawErrorPrevious);
  
  // Serial.print("DEBUG: [updateAttitudeControl] PID Constants - KP=");
  // Serial.print(ATTITUDE_KP);
  // Serial.print(", KI=");
  // Serial.print(ATTITUDE_KI);
  // Serial.print(", KD=");
  // Serial.println(ATTITUDE_KD);
  
  float pidOutput = ATTITUDE_KP * yawError + 
                   ATTITUDE_KI * yawErrorIntegral + 
                   ATTITUDE_KD * yawErrorDerivative;
  
  // Serial.print("DEBUG: [updateAttitudeControl] Calculated raw pidOutput=");
  // Serial.println(pidOutput);
  
  pidOutput = constrain(pidOutput, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
  
  // Serial.print("DEBUG: [updateAttitudeControl] After constraint, pidOutput=");
  // Serial.print(pidOutput);
  // Serial.print(", MAX_CONTROL_TORQUE=±");
  // Serial.println(MAX_CONTROL_TORQUE);
  
  int16_t scaledOutput = (int16_t)(pidOutput * 10);
  // Serial.print("DEBUG: [updateAttitudeControl] Scaled output for command=");
  // Serial.println(scaledOutput);
  
  sendBinaryCommand(CMD_TYPE_TORQUE, scaledOutput);
  
  // Serial.println("DEBUG: [updateAttitudeControl] Exit");
}

// MODIFIED: 5-byte command packets
void sendBinaryCommand(uint8_t cmdType, int16_t value) {
  uint8_t packet[5];
  packet[0] = PACKET_START_BYTE;
  packet[1] = cmdType;
  packet[2] = (uint8_t)(value & 0xFF);
  packet[3] = (uint8_t)(value >> 8);
  packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3];
  
  primarySerial.write(packet, sizeof(packet));
}

void processPrimaryPacket() {
  uint8_t packet[9];
  size_t n = primarySerial.readBytes(packet, 9);
  
  // Add debugging
  Serial.print("Received primary packet bytes: ");
  for (int i = 0; i < n; i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  if (n != 9) {
    Serial.println("Incomplete packet received");
    // Discard remaining bytes if packet is incomplete
    while (primarySerial.available()) {
      primarySerial.read();
    }
    return;
  }
  
  if (packet[0] != PACKET_START_BYTE || packet[8] != PACKET_END_BYTE) {
    Serial.println("Invalid start/end markers");
    return;
  }
  
  uint8_t calcChecksum = 0;
  for (int i = 0; i < 7; i++) {
    calcChecksum ^= packet[i];
  }
  
  if (calcChecksum == packet[7]) {
    // Store the old value for debug comparison
    int16_t oldRPM = currentWheelRPM;
    
    // Update the wheel values
    currentWheelRPM = packet[1] | (packet[2] << 8);
    currentWheelTorque = (float)(packet[3] | (packet[4] << 8)) / 100.0f;
    primaryStatus = packet[5];
    primaryFaults = packet[6];
    
    Serial.print("Updated RPM: ");
    Serial.print(oldRPM);
    Serial.print(" -> ");
    Serial.println(currentWheelRPM);
  } else {
    Serial.println("Checksum verification failed");
  }
}

// ---------------------------------------------------------------------------
// 8. Command Handling & Telemetry
// ---------------------------------------------------------------------------
void handleSerialCommands() {
  //Parse incoming rather than serial
  String cmdString = incoming;
  cmdString.trim();

  if (cmdString.startsWith("TARGET ")) {
    targetOrientation.yaw = cmdString.substring(7).toFloat();
    manualCommandActive = false;
  }
  else if (cmdString.startsWith("T")) {
    manualCommandActive = true;
    manualCommandTorque = cmdString.substring(1).toFloat();
    sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(manualCommandTorque * 100));
  }
  else if (cmdString.startsWith("S")) {
    manualCommandActive = true;
    manualCommandRPM = cmdString.substring(1).toFloat();
    sendBinaryCommand(CMD_TYPE_SPEED, (int16_t)manualCommandRPM);
  }
}

void sendLocalTelemetry() {
  Serial.print("Roll: ");
  Serial.print(currentOrientation.yaw, 1);
  Serial.print("° | Target: ");
  Serial.print(targetOrientation.yaw, 1);
  Serial.print("° | Torque ");
  Serial.print(currentWheelTorque, 1);
  Serial.print(" | Speed ");
  Serial.println(currentWheelRPM);
}

// New function to send telemetry over BLE
void sendBLETelemetry() {
  // Create a telemetry packet
  uint8_t telemetryPacket[20];  // Adjust size as needed
  int idx = 0;
  
  // Start marker
  telemetryPacket[idx++] = 0xFE;  // Telemetry start marker
  
  // Current orientation data (yaw as int16)
  int16_t yawInt = (int16_t)(currentOrientation.yaw * 100);  // Scale for precision
  telemetryPacket[idx++] = yawInt & 0xFF;
  telemetryPacket[idx++] = (yawInt >> 8) & 0xFF;
  
  // Target orientation data
  int16_t targetYawInt = (int16_t)(targetOrientation.yaw * 100);
  telemetryPacket[idx++] = targetYawInt & 0xFF;
  telemetryPacket[idx++] = (targetYawInt >> 8) & 0xFF;
  
  // Wheel RPM
  telemetryPacket[idx++] = currentWheelRPM & 0xFF;
  telemetryPacket[idx++] = (currentWheelRPM >> 8) & 0xFF;
  
  // Wheel torque (scaled)
  int16_t torqueInt = (int16_t)(currentWheelTorque * 100);
  telemetryPacket[idx++] = torqueInt & 0xFF;
  telemetryPacket[idx++] = (torqueInt >> 8) & 0xFF;
  
  // Status and faults
  telemetryPacket[idx++] = primaryStatus;
  telemetryPacket[idx++] = primaryFaults;
  
  // Manual mode flag
  telemetryPacket[idx++] = manualCommandActive ? 1 : 0;
  
  // End marker
  telemetryPacket[idx++] = 0xFF;
  
  // Write telemetry to ESP32
  telemetryChar.writeValue(telemetryPacket, idx);
}

// ---------------------------------------------------------------------------
// 9. Utility Functions
// ---------------------------------------------------------------------------
void updateLedBlink() {
  if (millis() - lastLedMillis >= LED_BLINK_INTERVAL_MS) {
    lastLedMillis = millis();
    digitalWrite(LED_PIN, (ledState = !ledState) ? HIGH : LOW);
  }
}