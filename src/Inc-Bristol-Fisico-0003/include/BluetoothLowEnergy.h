#ifndef BLUETOOTHLOWENERGY_H
#define BLUETOOTHLOWENERGY_H

#include <Arduino.h> 
// Libs do ESP32 para BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "FS.h"

// UUIDs dos serviços e características
#define TOTAL_ANGLE_SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TOTAL_ANGLE_CHARACTERISTIC_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PITCH_AND_ROLL_SERVICE_UUID         "16b26245-cb90-416b-a083-3a3c0e9271b0"
#define ROLL_CHARACTERISTIC_UUID            "44e1ccb2-a26f-4272-b726-15d4b0f221d4"
#define PITCH_CHARACTERISTIC_UUID           "0bb94e15-2437-49dc-a80d-310a1e725ec8"
#define BATTERY_SERVICE_UUID                "544c4044-46fe-463b-89ae-cac12926f83e"
#define BATTERY_CHARACTERISTIC_UUID         "cef2fac2-65b6-4a54-b52c-bf6585ac440a"

// OTA
#define OTA_SERVICE_UUID              "fb1e4001-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_RX        "fb1e4002-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_TX        "fb1e4003-54ae-4a28-9f74-dfccb248601d"

#define FORMAT_SPIFFS_IF_FAILED true
#define FORMAT_FFAT_IF_FAILED true

// Define which flash filesystem to use
//#define USE_SPIFFS  //comment to use FFat

#ifdef USE_SPIFFS
#define FLASH SPIFFS
#define FASTMODE false    //SPIFFS write is slow
#else
#define FLASH FFat
#define FASTMODE true    //FFat is faster
#endif

// OTA Mode definitions
#define NORMAL_MODE   0   // normal operation mode
#define UPDATE_MODE   1   // receiving firmware
#define OTA_MODE      2   // installing firmware

// Variáveis globais (extern)
extern BLECharacteristic *pAngulo;
extern BLECharacteristic *pRoll;
extern BLECharacteristic *pPitch;
extern BLECharacteristic *pBat;
extern BLECharacteristic *pCharacteristicTX;
extern BLECharacteristic *pCharacteristicRX;

extern bool deviceConnected, sendMode, sendSize;
extern bool writeFile, request;
extern int writeLen, writeLen2;
extern bool current;
extern int parts, next, cur, MTU;
extern int MODE;
extern unsigned long rParts, tParts;
extern unsigned long lastMillis;
extern uint8_t updater[16384];
extern uint8_t updater2[16384];
extern bool pauseNotifications;

extern unsigned long lastOtaNotification;
extern unsigned long lastSensorNotification;
extern unsigned long OTA_NOTIFICATION_PRIORITY; // ms de prioridade para OTA


// Callback classes
class MyServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};

class MyCallbacks : public BLECharacteristicCallbacks {
public:
    void onNotify(BLECharacteristic *pCharacteristic) override;
    void onWrite(BLECharacteristic *pCharacteristic) override;
};

// Function declarations
void setupBLE();
void sendAngleValue(float angle);
void sendPitchAndRoll(float pitch, float roll);
void sendBatteryPercentage(float percentage);
void blinkBleLed();
void writeBinary(fs::FS &fs, const char * path, uint8_t *dat, int len);
void sendOtaResult(String result);
void performUpdate(Stream &updateSource, size_t updateSize); 
void rebootEspWithReason(String reason);
void updateFromFS(fs::FS &fs);
void registerOtaNotification();

#endif // BLUETOOTHLOWENERGY_H