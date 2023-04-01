#include <BLE2902.h>
#include <M5StickC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <RTClib.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#define BUTTON_A_PIN 37

#define SAMPLING_RATE 50 // Hz

#define SERVICE_UUID "4faf0000-1fb5-459e-8fcc-c5c9c331914b"
#define ACC_CHAR_UUID "4faf0001-1fb5-459e-8fcc-c5c9c331914b"
#define GYRO_CHAR_UUID "4faf0002-1fb5-459e-8fcc-c5c9c331914b"
#define ATT_CHAR_UUID "4faf0003-1fb5-459e-8fcc-c5c9c331914b"

Madgwick filter;
unsigned long microsPerReading = 1000000 / SAMPLING_RATE;

float ax, ay, az, gx, gy, gz, mx, my, mz; // acc, gyro, and attitude

float batteryVoltage = 0.0F;

RTC_DS3231 rtc;

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pAccelChar = NULL;
BLECharacteristic* pGyroChar = NULL;
BLECharacteristic* pAttChar = NULL;

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
    // M5.Lcd.setCursor(0, 50);
    // M5.Lcd.println("  Pitch   Roll    Yaw");
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
        ACC_CHAR_UUID,
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

    // // attitude characteristic
    // pAttChar = pService->createCharacteristic(
    //     ATT_CHAR_UUID,
    //     BLECharacteristic::PROPERTY_READ |
    //     BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pAttChar->addDescriptor(new BLE2902());
    
    pService->start();
    pServer->getAdvertising()->start();

    return true;
}

// void onButtonAdvertise() {
//     // reset advertising
//     pService->start();
//     pServer->getAdvertising()->start();
// }

void setup() {
    // while(!Serial);

    M5.begin();
    setupUI();
    M5.IMU.Init();
    Wire.begin();
    rtc.begin();

    // attachInterrupt(digitalPinToInterrupt(BUTTON_A_PIN), onButtonAdvertise, RISING);

    initBLEServer();
    
    // BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->addServiceUUID(pService->getUUID());
    // pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06);
    // pAdvertising->setMaxPreferred(0x12);
    // BLEDevice::startAdvertising();
}

void loop() {
    
    // SENSOR READINGS
    static unsigned long prevMicros = 0;
    if (micros() - prevMicros >= microsPerReading) {

        // Read accelerometer and gyroscope data from IMU
        M5.IMU.getAccelData(&ax, &ay, &az);
        M5.IMU.getGyroData(&gx, &gy, &gz);
        M5.IMU.getAhrsData(&mx, &my, &mz);

        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        // filter.updateIMU(gx, gy, gz, ax, ay, az); // I wonder if this one is better

        // Get the quaternion values
        float roll = filter.getRollRadians();
        float pitch = filter.getPitchRadians();
        float yaw = filter.getYawRadians();

        // Compute linear acceleration in the world frame
        float world_ax = cos(yaw) * cos(pitch) * ax + (sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll)) * ay + (sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll)) * az;
        float world_ay = -sin(yaw) * cos(pitch) * ax + (cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll)) * ay + (cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)) * az;
        float world_az = sin(pitch) * ax - cos(pitch) * sin(roll) * ay - cos(pitch) * cos(roll) * az;
        
        // subtract the gravity from the world frame acceleration (assume gravity = 1g in the Z-direction)
        world_az += 1.0;

        if (deviceConnected) {

            // uint32_t timestamp = rtc.now().unixtime() + milliAdjustment;
            uint32_t microseconds = micros();
            
            // Convert data to byte arrays
            uint8_t accelData[16];
            memcpy(&accelData[0], &world_ax, sizeof(world_ax));
            memcpy(&accelData[4], &world_ay, sizeof(world_ay));
            memcpy(&accelData[8], &world_az, sizeof(world_az));
            memcpy(&accelData[12], &microseconds, sizeof(microseconds));
            pAccelChar->setValue(accelData, sizeof(accelData));
            pAccelChar->notify();
            
            uint8_t gyroData[16];
            memcpy(&gyroData[0], &gx, sizeof(gx));
            memcpy(&gyroData[4], &gy, sizeof(gy));
            memcpy(&gyroData[8], &gz, sizeof(gz));
            memcpy(&gyroData[12], &microseconds, sizeof(microseconds));
            pGyroChar->setValue(gyroData, sizeof(gyroData));
            pGyroChar->notify();

            // uint8_t attData[16];
            // memcpy(&attData[0], &pitch, sizeof(pitch));
            // memcpy(&attData[4], &roll, sizeof(roll));
            // memcpy(&attData[8], &yaw, sizeof(yaw));
            // memcpy(&attData[12], &microseconds, sizeof(microseconds));
            // pAttChar->setValue(attData, sizeof(attData));
            // pAttChar->notify();
        }

        prevMicros = micros();
    }

    // BATTERY CHECK
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck >= 1000) {
        // you want somewhere in between 4.2 and 3.7
        batteryVoltage = M5.Axp.GetBatVoltage();
        if (batteryVoltage < 3.7) {
            M5.Axp.PowerOff();
        }

        lastBatteryCheck = millis();
    }

    // UI UPDATE
    static unsigned long lastUIUpdate = 0;
    if (millis() - lastUIUpdate >= 500) {
        M5.Lcd.setCursor(0, 0);
        if (deviceConnected) {
            M5.Lcd.println("IMU TEST (connected)");
        } else {
            M5.Lcd.println("IMU TEST (not connected)");
        }
        
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("%6.2f  %6.2f  %6.2f o/s\n", gx, gy, gz);
        M5.Lcd.printf(" %5.2f   %5.2f   %5.2f G\n\n\n\n", ax, ay, az);
        // M5.Lcd.printf(" %5.2f   %5.2f   %5.2f\n", pitch, roll, yaw);

        M5.Lcd.setCursor(0, 70);
        M5.Lcd.printf("Battery: %.2f, Power: %.2f", batteryVoltage, M5.Axp.GetBatPower()/1000);

        lastUIUpdate = millis();
    }
}