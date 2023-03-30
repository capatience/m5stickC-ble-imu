#include <BLE2902.h>
#include <M5StickC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <RTClib.h>
#include <Wire.h>

#define BUTTON_A_PIN 37

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ACCL_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define GYRO_CHAR_UUID "beb5483f-36e1-4688-b7f5-ea07361b26a8"

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float batteryVoltage = 0.0F;

RTC_DS3231 rtc;

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pAccelChar = NULL;
BLECharacteristic* pGyroChar = NULL;

bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        pServer->getAdvertising()->start();
    };
};

void setupUI() {
    M5.Lcd.setRotation(3);
    M5.Lcd.setCursor(40, 0);
    M5.Lcd.println("IMU TEST");
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.println("   X       Y       Z");
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.println("  Pitch   Roll    Yaw");
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("Battery:");
}

bool initBLEServer() {
    BLEDevice::init("M5Stick-C IMU");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    pService = pServer->createService(SERVICE_UUID);
    
    // acceleration characteristic
    pAccelChar = pService->createCharacteristic(
        ACCL_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAccelChar->addDescriptor(new BLE2902());

    // gyro characteristic
    pGyroChar = pService->createCharacteristic(
        GYRO_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pGyroChar->addDescriptor(new BLE2902());

    pService->start();
    pServer->getAdvertising()->start();

    return true;
}

void onButtonAdvertise() {
    // reset advertising
    pService->start();
    pServer->getAdvertising()->start();
}

void setup() {
    // while(!Serial);

    M5.begin();
    setupUI();
    M5.IMU.Init();
    attachInterrupt(digitalPinToInterrupt(BUTTON_A_PIN), onButtonAdvertise, RISING);

    initBLEServer();
    
    Wire.begin();
    rtc.begin();
    
    // BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->addServiceUUID(pService->getUUID());
    // pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06);
    // pAdvertising->setMaxPreferred(0x12);
    // BLEDevice::startAdvertising();
}

void loop() {
    // Read accelerometer and gyroscope data from IMU
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("%6.2f  %6.2f  %6.2f o/s\n", gyroX, gyroY, gyroZ);
    M5.Lcd.printf(" %5.2f   %5.2f   %5.2f G\n\n\n\n", accX, accY, accZ);
    M5.Lcd.printf(" %5.2f   %5.2f   %5.2f\n", pitch, roll, yaw);

    // you want somewhere in between 4.2 and 3.7
    batteryVoltage = M5.Axp.GetBatVoltage();
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("Battery: %.2f, Power: %.2f", batteryVoltage, M5.Axp.GetBatPower()/1000);
    if (batteryVoltage < 3.7) {
        M5.Axp.PowerOff(); // need to turn it on manually after this happens (rather than just plugging it in)
    }

    if (deviceConnected) {
        // uint32_t timestamp = rtc.now().unixtime() + milliAdjustment;
        uint32_t milliseconds = millis();

        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("IMU TEST (connected)");
        
        // Convert data to byte arrays
        uint8_t accelData[16];
        memcpy(&accelData[0], &accX, sizeof(accX));
        memcpy(&accelData[4], &accY, sizeof(accY));
        memcpy(&accelData[8], &accZ, sizeof(accZ));
        memcpy(&accelData[12], &milliseconds, sizeof(milliseconds));
        pAccelChar->setValue(accelData, sizeof(accelData));
        pAccelChar->notify();
        
        uint8_t gyroData[16];
        memcpy(&gyroData[0], &gyroX, sizeof(gyroX));
        memcpy(&gyroData[4], &gyroY, sizeof(gyroY));
        memcpy(&gyroData[8], &gyroZ, sizeof(gyroZ));
        memcpy(&gyroData[12], &milliseconds, sizeof(milliseconds));
        pGyroChar->setValue(gyroData, sizeof(gyroData));
        pGyroChar->notify();
    } else {
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("IMU TEST (not connected)");
    }
    delay(50);
}