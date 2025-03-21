#ifndef BLUETOOTHLOWENERGY_H // Guarda para evitar inclusão múltipla
#define BLUETOOTHLOWENERGY_H

#include <Arduino.h> 
// Libs do ESP32 para BLE
#include <BLEDevice.h> // BLEDevice: Gerencia o dispositivo BLE
#include <BLEServer.h> // BLEServer: Cria e gerencia o servidor BLE (periférico)
#include <BLEUtils.h> // BLEUtils: Auxilia com operações BLE
#include <BLE2902.h> // BLE2902: Descritor padrão para notificações/indicações

// UUIDs do serviço e característica (precisam ser únicos)
#define TOTAL_ANGLE_SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TOTAL_ANGLE_CHARACTERISTIC_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PITCH_AND_ROLL_SERVICE_UUID         "16b26245-cb90-416b-a083-3a3c0e9271b0"
#define ROLL_CHARACTERISTIC_UUID            "44e1ccb2-a26f-4272-b726-15d4b0f221d4"
#define PITCH_CHARACTERISTIC_UUID           "0bb94e15-2437-49dc-a80d-310a1e725ec8"
#define BATTERY_SERVICE_UUID                "544c4044-46fe-463b-89ae-cac12926f83e"
#define BATTERY_CHARACTERISTIC_UUID         "cef2fac2-65b6-4a54-b52c-bf6585ac440a"
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
void sendAngleValue(float angle);
void sendPitchAndRoll(float pitch, float roll);
void sendBatteryPercentage(float percentage);
void blinkBleLed();

#endif // BLUETOOTHLOWENERGY_H
