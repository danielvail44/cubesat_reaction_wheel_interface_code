/******************************************************************************
 * CubeSat Reaction Wheel — Secondary (Command & Control) Board Code
 * [Integrated Version with Bluetooth + UART Protocol]
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "ArduinoBLE.h"
#include <Arduino_LSM6DS3.h>

// ---------------------------------------------------------------------------
// 1. Pin Definitions and Constants
// ---------------------------------------------------------------------------
const int LED_PIN = LED_BUILTIN;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;
const float SAMPLE_TIME_S = 0.01f;
const unsigned long LED_BLINK_INTERVAL_MS = 50;
bool newBLEcommand = false;
String incoming;
  float ax, ay, az, gx, gy, gz;
  float yaw = 0.0f;
unsigned long lastYawUpdateTime = 0;
bool initAttitudeControl = 0;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CONTROL_ENABLE true
#define TARGET_YAW 0.0f
#define MAX_CONTROL_TORQUE 100.0f

const float ATTITUDE_KP = 50.0f;
const float ATTITUDE_KI = 5.0f;
const float ATTITUDE_KD = 10.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;

// BLE Components
BLEDevice peripheral;
BLECharacteristic targetChar;

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

// ---------------------------------------------------------------------------
// 4. setup()
// ---------------------------------------------------------------------------
void setup() {

    delay(5000);
  Serial.begin(115200);
  delay(500);
    pinMode(7, OUTPUT);
  Serial.println("=== Reaction Wheel C&C Board (Integrated) ===");
  
  // BLE Initialization
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
    Serial.println("got here");
  BLE.scan();
Serial.println("got here");
  // UART Initialization
  primarySerial.begin(PRIMARY_BAUD);
  
  // IMU Initialization
  Wire.begin();
  Serial.println("got here");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
    Serial.println("got here");
  targetOrientation.yaw =TARGET_YAW;
  previousMicros = micros();
  
  pinMode(LED_PIN, OUTPUT);
  Serial.println("System Initialized");
}
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
            targetChar = discoveredDevice.characteristic(CHARACTERISTIC_UUID);
            if (targetChar && targetChar.canSubscribe()) {
              if (targetChar.subscribe()) {
                Serial.println("Subscribed to notifications");
                digitalWrite(7, HIGH);
                peripheral = discoveredDevice;
              }
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
    BLE.scan();
  }

  // Process New BLE Commands
  if (newBLEcommand) {
    Serial.print("New BLE Command: ");
    Serial.println(incoming);
    newBLEcommand = false;
    //Moved serial command handling into bluetooth section -TF 
    //Will require changes internally to handleSerialCommands to interpret "incoming" rather than the serial port itself
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
  }
}


// ---------------------------------------------------------------------------
// 6. IMU Functions
// ---------------------------------------------------------------------------

// Global variables (outside of function)

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

  currentOrientation.yaw = yaw;
}

/*
void calculateOrientation(float dt) {
  if (dt <= 0.0f || dt > 0.1f) dt = SAMPLE_TIME_S;
  
  float gyroRoll = previousRoll + gyroRates[0] * dt;
  float gyroPitch = previousPitch + gyroRates[1] * dt;
  
  currentOrientation.roll = COMPLEMENTARY_ALPHA * gyroRoll + 
                          (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.roll;
  currentOrientation.pitch = COMPLEMENTARY_ALPHA * gyroPitch + 
                           (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.pitch;
  
  previousRoll = currentOrientation.roll;
  previousPitch = currentOrientation.pitch;
  
  currentOrientation.yaw += gyroRates[2] * dt;
  currentOrientation.yaw = fmod(currentOrientation.yaw, 360.0f);
}
*/

// ---------------------------------------------------------------------------
// 7. Control & Communication (Modified Section)
// ---------------------------------------------------------------------------
void updateAttitudeControl() {
  if (!CONTROL_ENABLE) return;
  
  yawError = targetOrientation.yaw - currentOrientation.yaw;
  yawErrorIntegral += yawError * SAMPLE_TIME_S;
  yawErrorIntegral = constrain(yawErrorIntegral, -100.0f, 100.0f);
  
  float yawErrorDerivative = (yawError - yawErrorPrevious) / SAMPLE_TIME_S;
  yawErrorPrevious = yawError;
  
  float pidOutput = ATTITUDE_KP * yawError + 
                   ATTITUDE_KI * yawErrorIntegral + 
                   ATTITUDE_KD * yawErrorDerivative;
  pidOutput = constrain(pidOutput, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
  
  sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(pidOutput * 100));
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
  
  if (n != 9) {
    // Discard remaining bytes if packet is incomplete.
    while (primarySerial.available()) {
      primarySerial.read();
    }
    return;
  }
  
  if (packet[0] != PACKET_START_BYTE || packet[8] != PACKET_END_BYTE) {
    return;
  }
  
  uint8_t calcChecksum = 0;
  for (int i = 0; i < 7; i++) {
    calcChecksum ^= packet[i];
  }
  
  if (calcChecksum == packet[7]) {
    currentWheelRPM = packet[1] | (packet[2] << 8);
    currentWheelTorque = (float)(packet[3] | (packet[4] << 8)) / 100.0f;
    primaryStatus = packet[5];
    primaryFaults = packet[6];
  }
}

// ---------------------------------------------------------------------------
// 8. Command Handling & Telemetry
// ---------------------------------------------------------------------------
void handleSerialCommands() {
    //Parse incoming rather than serial
    String cmdString = incoming;
    cmdString.trim();

    //Removed Telemetry Send back for now due to complexity to send back through bluetooth - talk to team

    //if (cmdString.equalsIgnoreCase("GET")) {
    //sendLocalTelemetry();
    //}

    //Removed due to control over inputs
    //if (cmdString.equalsIgnoreCase("AUTO")) {
    //  manualCommandActive = false;
    //}
    if (cmdString.startsWith("TARGET ")) {
      targetOrientation.yaw = cmdString.substring(7).toFloat();
      manualCommandActive = false;
      sendBinaryCommand(CMD_TYPE_SPEED, 7000);
      delay(150);
      sendBinaryCommand(CMD_TYPE_TORQUE, 0);
    }
    else if (cmdString.startsWith("T")) {
      manualCommandActive = true;
      manualCommandTorque = cmdString.substring(2).toFloat();
      sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(manualCommandTorque * 100));
    }
    else if (cmdString.startsWith("S")) {
      manualCommandActive = true;
      manualCommandTorque = cmdString.substring(2).toFloat();
      manualCommandRPM = cmdString.substring(2).toFloat();
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

// ---------------------------------------------------------------------------
// 9. Utility Functions
// ---------------------------------------------------------------------------
void updateLedBlink() {
  if (millis() - lastLedMillis >= LED_BLINK_INTERVAL_MS) {
    lastLedMillis = millis();
    digitalWrite(LED_PIN, (ledState = !ledState) ? HIGH : LOW);
  }
}
