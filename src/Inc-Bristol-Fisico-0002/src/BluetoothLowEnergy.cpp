#include "BluetoothLowEnergy.h"

// Maiores definições sobre as classes e funções podem
// ser encontradas em include/BluetoothLowEnergy.h

// Globais
BLECharacteristic *pAngulo;
BLECharacteristic *pRoll;
BLECharacteristic *pPitch;
BLECharacteristic *pBat;
bool deviceConnected = false;
unsigned long lastMillis = 0;

// Callbacks do servidor BLE
void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Dispositivo conectado!");
}
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Dispositivo desconectado!");
    pServer->startAdvertising();
}

// Configuração do BLE
void setupBLE() {
    /* 
    init(): Inicializa o dispositivo BLE com nome personalizado
    createServer(): Cria uma instância do servidor BLE
    setCallbacks(): Registra os callbacks de conexão/desconexão
    */
    BLEDevice::init("Inclinômetro Bristol 0001");  // Nome do dispositivo BLE
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Cria o serviço BLE
    // Cria um novo serviço para o ângulo total
    BLEService *pAngleService = pServer->createService(TOTAL_ANGLE_SERVICE_UUID); // UUID definido no header

    // Cria um novo característica para o nível da bateria
    BLEService *pBatteryService = pServer->createService(BATTERY_SERVICE_UUID);
    pBat = pBatteryService->createCharacteristic(
        BATTERY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pBat->addDescriptor(new BLE2902());
    pBatteryService->start();

    // Cria a característica BLE para o ângulo total
    pAngulo = pAngleService->createCharacteristic(
        TOTAL_ANGLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    // Adiciona um descritor para notificações
    pAngulo->addDescriptor(new BLE2902());
    pAngleService->start();

    // Cria serviço separado para pitch e roll
    BLEService *pOrientationService = pServer->createService(PITCH_AND_ROLL_SERVICE_UUID);
    
    // Cria as características BLE para pitch e roll
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
    
    // Inicia o serviço
    pOrientationService->start();

    // Configurar advertising para mostrar ambos os serviços
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(TOTAL_ANGLE_SERVICE_UUID);
    pAdvertising->addServiceUUID(PITCH_AND_ROLL_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();

    /*
    Serial.println("BLE configurado com dois serviços:");
    Serial.println("1. Serviço de Ângulo Total");
    Serial.println("2. Serviço de Orientação (Pitch/Roll)");
    Serial.println("Aguardando conexão...");
    */
}

void sendAngleValue(float angle) {
    if (deviceConnected) {
        char txString[8];
        dtostrf(angle, 1, 1, txString);
        pAngulo->setValue(txString);
        pAngulo->notify();
        //Serial.printf("Enviando ângulo total: %.1f\n", angle);
    }
}

void sendPitchAndRoll(float pitch, float roll) {
    if (deviceConnected) {
        roll -= 5.0f; // Ajuste de offset
        pitch += 3.0f; // Ajuste de offset (TODO: Ajustar empiricamente depois de ter um case pronto)
        char pitchString[8];
        char rollString[8];
        dtostrf(pitch, 1, 1, pitchString);
        dtostrf(roll, 1, 1, rollString);
        
        pPitch->setValue(pitchString);
        pPitch->notify();
        pRoll->setValue(rollString);
        pRoll->notify();
        
        //Serial.printf("Enviando - Pitch: %.1f, Roll: %.1f\n", pitch, roll);
    }
}

void sendBatteryPercentage(float percentage) {
    if (deviceConnected) {
        char batString[8];
        dtostrf(percentage, 1, 1, batString); 
        pBat->setValue(batString);
        pBat->notify();
        //Serial.printf("Enviando nível de bateria: %.1f%%\n", percentage);
    }
}