#include <Arduino.h>
#include <PID_v1.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// === Pin Definitions ===
#define DEBUG_LED_PIN 7
#define BUTTON_PIN 10
#define NUM_CHANNELS 3

// === Watchdog Timeout ===
#define BLE_TIMEOUT_MS 30000
unsigned long lastBLEActivity = 0;

// === HeaterChannel Class ===
class HeaterChannel {
public:
  uint8_t heaterPin;
  uint8_t ntcPin;
  double inputTemp, outputPWM, targetTemp;
  bool pidEnabled;
  PID pid;
  BLECharacteristic *tempChar;
  BLECharacteristic *targetChar;
  BLECharacteristic *statusChar;

  HeaterChannel(uint8_t hPin, uint8_t nPin, double setpoint)
    : heaterPin(hPin), ntcPin(nPin), inputTemp(0), outputPWM(0),
      targetTemp(setpoint), pidEnabled(false),
      pid(&inputTemp, &outputPWM, &targetTemp, 2.0, 5.0, 1.0, DIRECT) {}

  void begin() {
    pinMode(heaterPin, OUTPUT);
    digitalWrite(heaterPin, LOW);
    pid.SetMode(AUTOMATIC);
  }

  void update() {
    inputTemp = readNTC(ntcPin);
    if (pidEnabled) {
      pid.Compute();
      digitalWrite(heaterPin, outputPWM > 0 ? HIGH : LOW);
    } else {
      digitalWrite(heaterPin, LOW);
    }
  }

  void updateBLE() {
    float temp = inputTemp;
    float target = targetTemp;
    tempChar->setValue(temp);
    targetChar->setValue(target);
    String mode = pidEnabled ? "PID" : "Manual";
    statusChar->setValue(mode);
  }

  void handleWrite(BLECharacteristic* characteristic) {
    String value = characteristic->getValue().c_str();
    if (characteristic == statusChar) {
      pidEnabled = (value == "PID");
    } else if (characteristic == targetChar) {
      targetTemp = value.toFloat();
    }
  }

private:
  double readNTC(uint8_t pin) {
    int raw = analogRead(pin);
    double resistance = (4095.0 / raw - 1.0) * 10000.0;
    double temperature = 1.0 / (log(resistance / 10000.0) / 3950.0 + 1.0 / (25.0 + 273.15)) - 273.15;
    return temperature;
  }
};

// === BLE Write Callback ===
class ChannelCallbacks : public BLECharacteristicCallbacks {
public:
  HeaterChannel* channel;
  ChannelCallbacks(HeaterChannel* ch) : channel(ch) {}

  void onWrite(BLECharacteristic* characteristic) override {
    channel->handleWrite(characteristic);
    lastBLEActivity = millis();  // Reset watchdog
  }
};

// === Globals ===
HeaterChannel* channels[NUM_CHANNELS] = {
  new HeaterChannel(4, 3, 40.0),
  new HeaterChannel(6, 1, 40.0),
  new HeaterChannel(5, 0, 40.0)
};

BLEServer *pServer;
BLECharacteristic *uptimeChar;
bool bleConnected = false;
unsigned long lastBlink = 0;
bool ledState = false;

// === BLE Server Callbacks ===
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    lastBLEActivity = millis();
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    lastBLEActivity = millis();
  }
};

// === BLE Setup ===
void setupBLE() {
  BLEDevice::init("HeatMat Controller");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *service = pServer->createService("12345678-1234-1234-1234-000000000000");

  for (int i = 0; i < NUM_CHANNELS; i++) {
    char tempUUID[37], targetUUID[37], modeUUID[37];
    sprintf(tempUUID,   "12345678-1234-1234-1234-00000000001%d", i);
    sprintf(targetUUID, "12345678-1234-1234-1234-00000000002%d", i);
    sprintf(modeUUID,   "12345678-1234-1234-1234-00000000003%d", i);

    channels[i]->tempChar = service->createCharacteristic(tempUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    channels[i]->targetChar = service->createCharacteristic(targetUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    channels[i]->statusChar = service->createCharacteristic(modeUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    channels[i]->tempChar->addDescriptor(new BLE2902());
    channels[i]->targetChar->addDescriptor(new BLE2902());
    channels[i]->statusChar->addDescriptor(new BLE2902());

    BLEDescriptor* tempLabel   = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    BLEDescriptor* targetLabel = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    BLEDescriptor* modeLabel   = new BLEDescriptor(BLEUUID((uint16_t)0x2901));

    tempLabel->setValue("Channel " + String(i + 1) + " Temperature (°C)");
    targetLabel->setValue("Channel " + String(i + 1) + " Target Temperature (°C)");
    modeLabel->setValue("Channel " + String(i + 1) + " Control Mode (PID/Manual)");

    channels[i]->tempChar->addDescriptor(tempLabel);
    channels[i]->targetChar->addDescriptor(targetLabel);
    channels[i]->statusChar->addDescriptor(modeLabel);

    channels[i]->targetChar->setCallbacks(new ChannelCallbacks(channels[i]));
    channels[i]->statusChar->setCallbacks(new ChannelCallbacks(channels[i]));
  }

  // Uptime characteristic
  uptimeChar = service->createCharacteristic("12345678-1234-1234-1234-00000000UPTM", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  uptimeChar->addDescriptor(new BLE2902());
  uptimeChar->addDescriptor(new BLEDescriptor(BLEUUID((uint16_t)0x2901)));
  uptimeChar->getDescriptorByUUID(BLEUUID((uint16_t)0x2901))->setValue("System Uptime (s)");

  service->start();
  pServer->getAdvertising()->start();
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i]->begin();
  }

  setupBLE();
}

// === Loop ===
void loop() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i]->update();
    channels[i]->updateBLE();
  }

  // Update uptime
  uint32_t uptime = millis() / 1000;
  uptimeChar->setValue(uptime);

  // Watchdog: disable heaters if BLE disconnected too long
  if (!bleConnected && millis() - lastBLEActivity > BLE_TIMEOUT_MS) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      channels[i]->pidEnabled = false;
    }
  }

  handleDebugLED();
  handleButton();
}

// === Debug LED ===
void handleDebugLED() {
  if (bleConnected) {
    digitalWrite(DEBUG_LED_PIN, HIGH);
  } else {
    if (millis() - lastBlink >= 1000) {
      ledState = !ledState;
      digitalWrite(DEBUG_LED_PIN, ledState);
      lastBlink = millis();
    }
  }
}

// === Re-pair Button ===
void handleButton() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);
  if (lastState == HIGH && currentState == LOW) {
    BLEDevice::deinit(true);
    delay(100);
    setupBLE();
  }
  lastState = currentState;
}