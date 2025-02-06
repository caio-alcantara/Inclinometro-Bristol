#include "BluetoothLowEnergy.h"
#include <Arduino.h>

// Maiores definições sobre as classes e funções podem
// ser encontradas em include/BluetoothLowEnergy.h

// Globais
BLECharacteristic *pAngulo;
bool deviceConnected = false;
uint32_t counter = 0;
unsigned long lastMillis = 0;

// Callbacks
void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising();
}

// Configuração inicial do serviço BLE
void setupBLE() {
    // Serial.begin(115200); -> Será iniciado no main.cpp
    /* 
    init(): Inicializa o dispositivo BLE com nome personalizado
    createServer(): Cria uma instância do servidor BLE
    setCallbacks(): Registra os callbacks de conexão/desconexão
    */
    BLEDevice::init("Inclinômetro Bristol 0001"); // Nome do dispositivo
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    

    // Cria o serviço BLE
    // Cria um novo serviço usando o UUID definido
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pAngulo = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY // Permite notificações (envio de dados contínuos)
    );
    
    pAngulo->addDescriptor(new BLE2902()); // Adiciona um descritor para notificações, a fim de habilitá-las no cliente (app)
    pService->start(); // Inicia o serviço
    pServer->getAdvertising()->start(); // Inicia o advertising (torna o dispositivo visível para conexão)
    
    Serial.println("Aguardando conexão...");
}

void sendAngleValue(float angle) {
    if (deviceConnected) {
        char txString[8];
        sprintf(txString, "%.1f", angle);
        pAngulo->setValue(txString);
        pAngulo->notify();
        
        Serial.print("Valor enviado: ");
        Serial.println(angle);
    }
}

// Atualização do contador e envio BLE
void updateCounterAndNotify() {
    if (deviceConnected) {
        if (millis() - lastMillis >= 100) {
            counter++;
            lastMillis = millis();
            
            char txString[8];
            sprintf(txString, "%d", counter);
            pAngulo->setValue(txString);
            pAngulo->notify();
            
            Serial.print("Valor enviado: ");
            Serial.println(counter);
        }
    }
}