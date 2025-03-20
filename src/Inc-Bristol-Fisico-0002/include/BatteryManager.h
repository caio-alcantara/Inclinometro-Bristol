#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

class BatteryManager {
public:
    struct BatteryData {
        float percentage;    // Porcentagem de carga (0-100%)
        float voltage;       // Tensão da bateria (V)
        unsigned long timestamp; // Timestamp da última leitura
    };

    BatteryManager(SFE_MAX1704X &fuelGauge, TwoWire &wire = Wire);
    
    bool begin();            // Inicializa o sensor
    bool update();           // Atualiza os dados da bateria
    const BatteryData& getData() const; // Retorna os dados
    bool isOK() const;       // Verifica se o sensor está operacional
    
    // Para debug/calibração
    void simulateData(float percentage, float voltage);

private:
    SFE_MAX1704X &_fuelGauge;
    TwoWire &_wire;
    BatteryData _data;
    bool _initialized = false;
};