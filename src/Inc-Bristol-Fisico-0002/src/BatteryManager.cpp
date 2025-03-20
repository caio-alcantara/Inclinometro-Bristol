#include "BatteryManager.h"

// Construtor (com Wire como padrão)
BatteryManager::BatteryManager(SFE_MAX1704X &fuelGauge, TwoWire &wire)
    : _fuelGauge(fuelGauge), _wire(wire) {}

// Inicialização do sensor
bool BatteryManager::begin() {
    _initialized = _fuelGauge.begin();
    if (_initialized) {
        _fuelGauge.quickStart(); // Calibração inicial
        update(); // Primeira leitura
    }
    return _initialized;
}

// Atualiza os dados da bateria
bool BatteryManager::update() {
    if (!_initialized) return false;

    _data.percentage = _fuelGauge.getSOC();
    _data.voltage = _fuelGauge.getVoltage();
    _data.timestamp = millis();
    
    return true;
}

// Retorna os dados atualizados
const BatteryManager::BatteryData& BatteryManager::getData() const {
    return _data;
}

// Verifica o status do sensor
bool BatteryManager::isOK() const {
    return _initialized;
}

// Simula dados para testes (sobrescreve valores reais)
void BatteryManager::simulateData(float percentage, float voltage) {
    _data.percentage = percentage;
    _data.voltage = voltage;
    _data.timestamp = millis();
}