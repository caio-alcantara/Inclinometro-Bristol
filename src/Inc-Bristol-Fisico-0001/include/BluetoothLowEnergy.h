#ifndef BLUETOOTHLOWENERGY_H // Guarda para evitar inclusão múltipla
#define BLUETOOTHLOWENERGY_H

// Libs do ESP32 para BLE
#include <BLEDevice.h> // BLEDevice: Gerencia o dispositivo BLE
#include <BLEServer.h> // BLEServer: Cria e gerencia o servidor BLE (periférico)
#include <BLEUtils.h> // BLEUtils: Auxilia com operações BLE
#include <BLE2902.h> // BLE2902: Descritor padrão para notificações/indicações

// UUIDs do serviço e característica (precisam ser únicos)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
/* 
SERVICE_UUID: Identifica o serviço principal
CHARACTERISTIC_UUID: Identifica a característica específica dentro do serviço
*/

// Variáveis globais (extern)
extern BLECharacteristic *pAngulo; // Ponteiro para a caracteristica BLE
extern bool deviceConnected; 
extern unsigned long lastMillis;

// Callbacks do servidor BLE
/* 
Herda de BLEServerCallbacks para tratar eventos:
onConnect(): Atualiza a flag quando um dispositivo se conecta
onDisconnect(): Reinicia o advertising (deixa o dispositivo visível para conexão) para permitir novas conexões
*/
class MyServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};

void setupBLE();
void updateCounterAndNotify();
void sendAngleValue(float angle);

#endif // BLUETOOTHLOWENERGY_H