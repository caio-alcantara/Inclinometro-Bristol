#include "BluetoothLowEnergy.h"
#include "Config.h"
#include "Arduino.h"

#include <Update.h>
#include "FS.h"
#include "FFat.h"
#include "SPIFFS.h"

unsigned long lastOtaNotification = 0;
unsigned long lastSensorNotification = 0;
unsigned long OTA_NOTIFICATION_PRIORITY = 100; // ms de prioridade para OTA

// Globais
BLECharacteristic *pAngulo;
BLECharacteristic *pRoll;
BLECharacteristic *pPitch;
BLECharacteristic *pBat;
bool deviceConnected = false, sendMode = false, sendSize = true;
bool writeFile = false, request = false;
int writeLen = 0, writeLen2 = 0;
bool current = true;
int parts = 0, next = 0, cur = 0, MTU = 0;
int MODE = NORMAL_MODE;
unsigned long rParts, tParts;
unsigned long lastMillis = 0;
uint8_t updater[16384];
uint8_t updater2[16384];
BLECharacteristic *pCharacteristicTX;
BLECharacteristic *pCharacteristicRX;

// Add a flag to control notifications during OTA
bool pauseNotifications = false;

// Callbacks do servidor BLE
void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Dispositivo conectado!");
    //digitalWrite(Config::BLE_LED_PIN, HIGH);
}
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Dispositivo desconectado!");
    pServer->startAdvertising();
    blinkBleLed();
}

void MyCallbacks::onNotify(BLECharacteristic *pCharacteristic) {
    uint8_t* pData;
    std::string value = pCharacteristic->getValue();
    int len = value.length();
    pData = pCharacteristic->getData();
    if (pData != NULL) {
        Serial.print("TX  ");
        for (int i = 0; i < len; i++) {
            Serial.printf("%02X ", pData[i]);
        }
        Serial.println();
    }
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t* pData;
    std::string value = pCharacteristic->getValue();
    int len = value.length();
    pData = pCharacteristic->getData();
    if (pData != NULL) {
        // Debug output for OTA commands
        Serial.print("RX  ");
        for (int i = 0; i < 5 && i < len; i++) {
            Serial.printf("%02X ", pData[i]);
        }
        Serial.println();
        
        if (pData[0] == 0xFB) {
            int pos = pData[1];
            for (int x = 0; x < len - 2; x++) {
                if (current) {
                    updater[(pos * MTU) + x] = pData[x + 2];
                } else {
                    updater2[(pos * MTU) + x] = pData[x + 2];
                }
            }
            
        } else if (pData[0] == 0xFC) {
            if (current) {
                writeLen = (pData[1] * 256) + pData[2];
            } else {
                writeLen2 = (pData[1] * 256) + pData[2];
            }
            current = !current;
            cur = (pData[3] * 256) + pData[4];
            writeFile = true;
            if (cur < parts - 1) {
                request = !FASTMODE;
            }
            Serial.printf("FC Command: Chunk %d/%d, len=%d\n", cur+1, parts, current ? writeLen2 : writeLen);
            
        } else if (pData[0] == 0xFD) {
            Serial.println("FD Command: Preparing for update");
            pauseNotifications = true; // Pause regular notifications during update
            sendMode = true;
            if (FLASH.exists("/update.bin")) {
                FLASH.remove("/update.bin");
            }
            
        } else if (pData[0] == 0xFE) {
            rParts = 0;
            tParts = (pData[1] * 256 * 256 * 256) + (pData[2] * 256 * 256) + (pData[3] * 256) + pData[4];

            Serial.print("FE Command: File Size Info - Available space: ");
            Serial.println(FLASH.totalBytes() - FLASH.usedBytes());
            Serial.print("File Size: ");
            Serial.println(tParts);

        } else if (pData[0] == 0xFF) {
            parts = (pData[1] * 256) + pData[2];
            MTU = (pData[3] * 256) + pData[4];
            MODE = UPDATE_MODE;
            Serial.printf("FF Command: Update Starting - Parts: %d, MTU: %d\n", parts, MTU);

        } else if (pData[0] == 0xEF) {
            Serial.println("EF Command: Format Flash");
            FLASH.format();
            sendSize = true;
        }
    }

    if (MODE == NORMAL_MODE && pData[0] != 0xFD) {
        // Se estamos no modo normal e não é um comando de início de OTA, garantir que notificações estejam habilitadas
        pauseNotifications = false;
    }
}

// Configuração do BLE
void setupBLE() {
    /* 
    init(): Inicializa o dispositivo BLE com nome personalizado
    createServer(): Cria uma instância do servidor BLE
    setCallbacks(): Registra os callbacks de conexão/desconexão
    */
    BLEDevice::init(Config::device_name);  // Nome do dispositivo BLE
    BLEDevice::setMTU(517); 
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create OTA service first (highest priority)
    BLEService *pOtaService = pServer->createService(OTA_SERVICE_UUID);
    pCharacteristicTX = pOtaService->createCharacteristic(
        CHARACTERISTIC_UUID_TX, 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicRX = pOtaService->createCharacteristic(
        CHARACTERISTIC_UUID_RX, 
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );

    pCharacteristicRX->setCallbacks(new MyCallbacks());
    pCharacteristicTX->setCallbacks(new MyCallbacks());
    pCharacteristicTX->addDescriptor(new BLE2902());
    pCharacteristicTX->setNotifyProperty(true);
    pOtaService->start();
    Serial.println("OTA Service Started");

    // Battery service
    BLEService *pBatteryService = pServer->createService(BATTERY_SERVICE_UUID);
    pBat = pBatteryService->createCharacteristic(
        BATTERY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pBat->addDescriptor(new BLE2902());
    pBatteryService->start();
    Serial.println("Battery Service Started");

    // Angle service
    BLEService *pAngleService = pServer->createService(TOTAL_ANGLE_SERVICE_UUID);
    pAngulo = pAngleService->createCharacteristic(
        TOTAL_ANGLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pAngulo->addDescriptor(new BLE2902());
    pAngleService->start();
    Serial.println("Angle Service Started");

    BLE2902* p2902 = new BLE2902();
    p2902->setNotifications(true);  // Ativar notificações explicitamente
    pAngulo->addDescriptor(p2902);

    // Orientation service
    BLEService *pOrientationService = pServer->createService(PITCH_AND_ROLL_SERVICE_UUID);
    pRoll = pOrientationService->createCharacteristic(
        ROLL_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pRoll->addDescriptor(new BLE2902());

    pPitch = pOrientationService->createCharacteristic(
        PITCH_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pPitch->addDescriptor(new BLE2902());
    
    pOrientationService->start();
    Serial.println("Orientation Service Started");

    // Configure advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(OTA_SERVICE_UUID);         // Add OTA service first
    pAdvertising->addServiceUUID(TOTAL_ANGLE_SERVICE_UUID);
    pAdvertising->addServiceUUID(PITCH_AND_ROLL_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    Serial.println("BLE Advertising Started");
}

void sendAngleValue(float angle) {
    // Verifica se estamos no modo normal e se há conexão
    if (MODE != NORMAL_MODE || !deviceConnected) {
        Serial.println("Not in normal mode or device not connected");
        return;
    }

    unsigned long currentTime = millis();
    
    Serial.println("Sending angle value: " + String(angle));

    // Se estamos em modo OTA ou se uma notificação OTA foi enviada recentemente, reduz a frequência
    if (pauseNotifications) {
        // Durante OTA, envia dados com frequência muito reduzida (a cada 1 segundo)
        if (currentTime - lastSensorNotification < 1000) {
            Serial.println("Skipping angle notification during OTA");
            return;
        }
    } else {
        // Em modo normal, verifica se uma notificação OTA foi enviada recentemente
        if (currentTime - lastOtaNotification < OTA_NOTIFICATION_PRIORITY) {
            Serial.println("Skipping angle notification due to recent OTA");
            return; // Dá prioridade para notificações OTA
        }
        
        // Em modo normal, limita a frequência a cada 100ms
        /*if (currentTime - lastSensorNotification < 100) {
            Serial.println("Skipping angle notification due to frequency limit");
            return;
        }*/
    }

    // Registra o tempo da última notificação de sensor
    //lastSensorNotification = currentTime;
    
    char txString[8];
    dtostrf(angle, 1, 1, txString);
    pAngulo->setValue(txString);
    pAngulo->notify();
    
    // Delay reduzido para minimizar bloqueio
    delay(5);
}


void sendPitchAndRoll(float pitch, float roll) {
    // Only send if notifications aren't paused and we're in normal mode
    if (pauseNotifications || MODE != NORMAL_MODE) {
        return;
    }

    if (deviceConnected) {
        roll -= 5.0f; // Ajuste de offset
        pitch += 3.0f; // Ajuste de offset
        
        char pitchString[8];
        char rollString[8];
        dtostrf(pitch, 1, 1, pitchString);
        dtostrf(roll, 1, 1, rollString);
        
        pPitch->setValue(pitchString);
        pPitch->notify();
        delay(10); // Small delay between notifications
        
        pRoll->setValue(rollString);
        pRoll->notify();
        delay(10); // Small delay after notifications
    }
}

void sendBatteryPercentage(float percentage) {
    // Only send if notifications aren't paused and we're in normal mode
    if (pauseNotifications || MODE != NORMAL_MODE) {
        return;
    }

    if (deviceConnected) {
        char batString[8];
        dtostrf(percentage, 1, 1, batString); 
        pBat->setValue(batString);
        pBat->notify();
        delay(10); // Small delay after notification
    }
}

void blinkBleLed() {
    static unsigned long previousMillis = 0;
    const unsigned long interval = 150;

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        // Pisca o LED para indicar erro
        digitalWrite(Config::BLE_LED_PIN, !digitalRead(Config::BLE_LED_PIN));
    }
}

void writeBinary(fs::FS &fs, const char * path, uint8_t *dat, int len) {
    File file = fs.open(path, FILE_APPEND);
  
    if (!file) {
        Serial.println("- failed to open file for writing");
        return;
    }
    file.write(dat, len);
    file.close();
    writeFile = false;
    rParts += len;
    Serial.printf("Writing %d bytes to file, total: %lu/%lu\n", len, rParts, tParts);
}

void sendOtaResult(String result) {
    lastOtaNotification = millis(); // Registra o tempo da notificação OTA
    pCharacteristicTX->setValue(result.c_str());
    pCharacteristicTX->notify();
    delay(100); 
}

void registerOtaNotification() {
    lastOtaNotification = millis();
}

void performUpdate(Stream &updateSource, size_t updateSize) {
    char s1 = 0x0F;
    String result = String(s1);
    
    // Make sure notifications are paused during update
    pauseNotifications = true;

    if (Update.begin(updateSize)) {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize) {
            Serial.println("Written: " + String(written) + " successfully");
        }
        else {
            Serial.println("Written only: " + String(written) + "/" + String(updateSize) + ". Retry?");
        }
        result += "Written: " + String(written) + "/" + String(updateSize) + " [" + String((written / updateSize) * 100) + "%] \n";
        
        if (Update.end()) {
            Serial.println("OTA done!");
            result += "OTA Done: ";
            if (Update.isFinished()) {
                Serial.println("Update successfully completed. Rebooting...");
                result += "Success!\n";
            }
            else {
                Serial.println("Update not finished? Something went wrong!");
                result += "Failed!\n";
            }
        }
        else {
            Serial.println("Error Occurred. Error #: " + String(Update.getError()));
            result += "Error #: " + String(Update.getError());
        }
    }
    else {
        Serial.println("Not enough space to begin OTA");
        result += "Not enough space for OTA";
    }
    
    if (deviceConnected) {
        sendOtaResult(result);
        delay(1000); // Give time for notification to be sent
    }
}

void rebootEspWithReason(String reason) {
    Serial.println(reason);
    delay(1000);
    ESP.restart();
}

void updateFromFS(fs::FS &fs) {
    File updateBin = fs.open("/update.bin");
    if (updateBin) {
        if (updateBin.isDirectory()) {
            Serial.println("Error, update.bin is not a file");
            updateBin.close();
            pauseNotifications = false; // Resume notifications if update fails
            return;
        }
  
        size_t updateSize = updateBin.size();
  
        if (updateSize > 0) {
            Serial.println("Starting update process");
            performUpdate(updateBin, updateSize);
        }
        else {
            Serial.println("Error, file is empty");
            pauseNotifications = false; // Resume notifications if update fails
        }
  
        updateBin.close();
  
        // Remove the binary file after update
        Serial.println("Removing update file");
        fs.remove("/update.bin");
  
        rebootEspWithReason("Rebooting to complete OTA update");
    }
    else {
        Serial.println("Could not load update.bin from spiffs root");
        // Resume notifications if update fails
        pauseNotifications = false;
    }
}