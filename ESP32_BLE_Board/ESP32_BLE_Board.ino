#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs that match what the Nano is looking for
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Send commands TO Nano
#define TELEMETRY_CHAR_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a9"  // Receive telemetry FROM Nano

BLEServer* pServer = NULL;
BLECharacteristic* pCommandChar = NULL;    // For sending commands to Nano
BLECharacteristic* pTelemetryChar = NULL;  // For receiving telemetry from Nano
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Serial buffer for commands
String serialCommand = "";

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
  }
};

class TelemetryCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("TELEMETRY:");
      // Forward raw bytes after the TELEMETRY: prefix
      for (int i = 0; i < value.length(); i++) {
        Serial.write(value[i]);
      }
      Serial.print('\n');  // Add newline directly
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("Starting ESP32 BLE Bridge (Bidirectional)");

  // Create the BLE Device
  BLEDevice::init("ESP32_ReactionWheel_Bridge");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the Command Characteristic (ESP32 -> Nano)
  pCommandChar = pService->createCharacteristic(
                      COMMAND_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create a BLE Descriptor for command notifications
  BLE2902* p2902Descriptor = new BLE2902();
  p2902Descriptor->setNotifications(true);
  pCommandChar->addDescriptor(p2902Descriptor);

  // Create the Telemetry Characteristic (Nano -> ESP32)
  pTelemetryChar = pService->createCharacteristic(
                      TELEMETRY_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Set callbacks for telemetry writes
  pTelemetryChar->setCallbacks(new TelemetryCharacteristicCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("Waiting for BLE client connection...");
}

void loop() {
  // Read from Serial (commands from PC) and send over BLE
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      // Send the command over BLE when we get a newline
      if (deviceConnected && serialCommand.length() > 0) {
        // First update the characteristic value
        pCommandChar->setValue(serialCommand.c_str());
        
        // Then send notification
        pCommandChar->notify();
        
        Serial.print("Sent BLE command: '");
        Serial.print(serialCommand);
        Serial.println("'");
      }
      serialCommand = "";
    } else {
      // Build up the command string
      serialCommand += c;
    }
  }

  // Handle disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Client disconnected - Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  // Handle initial connection
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("Client fully connected");
    oldDeviceConnected = deviceConnected;
  }
  
  delay(10);
}