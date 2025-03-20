// ******************* Imports e bibliotecas ******************* //
#include <Arduino.h>
#include <MPU9250.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "Config.h"
#include "KalmanFilter.h"
#include "SensorManager.h"
#include "DataProcessor.h"
#include "BluetoothLowEnergy.h"

// ******************* Instanciação de objetos e Variáveis Globais  ******************* //
MPU9250 mpu;
MPU9250Setting mpu_settings;
TwoWire wire = Wire;
SensorManager sensor(mpu, mpu_settings, Wire);
SFE_MAX1704X lipo; // Declarado globalmente



// ******************* Variáveis de Temporização ******************* //
unsigned long lastSensorUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastBLEUpdate = 0;

// ******************* Variáveis de Estado ******************* //
float latestFilteredPitch = 0.0;
float latestFilteredRoll = 0.0;
float latestTotalInclination = 0.0;
float batteryPercentage = 0.0;

// ******************* Filtros Kalman ******************* //
KalmanFilter pitch_filter(
    Config::Kalman::PITCH_Q_ANGLE,
    Config::Kalman::PITCH_Q_BIAS,
    Config::Kalman::PITCH_R_MEASURE
);

KalmanFilter roll_filter(
    Config::Kalman::ROLL_Q_ANGLE,
    Config::Kalman::ROLL_Q_BIAS,
    Config::Kalman::ROLL_R_MEASURE
);

void scanI2C() {
    byte error, address;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Dispositivo encontrado: 0x");
            Serial.println(address, HEX);
        }
    }
}

// ******************* Funções de Tratamento de Erro ******************* //
void handleSensorError() {
    Serial.println("Erro crítico no sensor! Tentando reconectar...");
    while (true) {
        digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        delay(100);
        if (sensor.initialize()) {
            digitalWrite(Config::LED_PIN, LOW);
            Serial.println("Sensor reconectado!");
            break;
        }
    }
}

void handleBatteryError() {
    Serial.println("Erro crítico na bateria! Tentando reconectar...");
    while (true) {
        digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        delay(10);
        if (lipo.begin()) {
            digitalWrite(Config::LED_PIN, LOW);
            Serial.println("Bateria reconectada!");
            lipo.quickStart();
            break;
        }
    }
}

// ******************* Inicialização dos Filtros ******************* //
void initializeFilters() {
    const auto data = sensor.getData();
    pitch_filter.initialize(DataProcessor::calculateAccelerometerPitch(
        data.accel.x, data.accel.y, data.accel.z));
    roll_filter.initialize(DataProcessor::calculateAccelerometerRoll(
        data.accel.y, data.accel.z));
}

// ******************* Setup ******************* //
void setup() {
    Serial.begin(Config::SERIAL_BAUD_RATE);
    Wire.begin(21, 22);
    pinMode(Config::LED_PIN, OUTPUT);

    // Inicializa MAX1704X
    if (!lipo.begin()) handleBatteryError();
    lipo.quickStart();

    // Inicializa MPU9250
    if(!sensor.initialize()) handleSensorError();

    setupBLE();
    sensor.calibrateAccelGyro();
    initializeFilters();
    lastSensorUpdate = millis();

    scanI2C();
}

// ******************* Loop Principal Corrigido ******************* //
void loop() {
    unsigned long currentMillis = millis();

    // Atualização do Sensor (executa o mais rápido possível)
    if (sensor.update()) {
        const float delta_time = (currentMillis - lastSensorUpdate) / 1000.0f;
        lastSensorUpdate = currentMillis;

        const auto data = sensor.getData();

        // Cálculos dos ângulos
        const float pitch_acc = DataProcessor::calculateAccelerometerPitch(
            data.accel.x, data.accel.y, data.accel.z);
        const float roll_acc = DataProcessor::calculateAccelerometerRoll(
            data.accel.y, data.accel.z);

        // Atualização dos filtros
        latestFilteredPitch = pitch_filter.update(pitch_acc, data.gyro.y, delta_time);
        latestFilteredRoll = roll_filter.update(roll_acc, data.gyro.x, delta_time);
        latestTotalInclination = DataProcessor::calculateTotalInclination(
            latestFilteredPitch, latestFilteredRoll);

        // Debugging
        //Serial.printf("Pitch: %.2f | Roll: %.2f | Inclinação: %.2f\n", 
        //             latestFilteredPitch, latestFilteredRoll, latestTotalInclination);
    }

    // Atualização da Bateria a cada 500ms
    if (currentMillis - lastBatteryUpdate >= 500) {
        if (!lipo.begin()) {
            handleBatteryError();
        } else {
            float batteryVoltage = lipo.getVoltage();
            batteryPercentage = lipo.getSOC();
            Serial.println(lipo.getVoltage());
            Serial.println("----------------");
            Serial.printf("Bateria: %.2f%%\n", batteryPercentage);
            lastBatteryUpdate = currentMillis;
        }
    }

    // Atualização BLE a cada 100ms
    if (currentMillis - lastBLEUpdate >= 100) {
        sendBatteryPercentage(batteryPercentage);
        sendAngleValue(latestTotalInclination);
        sendPitchAndRoll(latestFilteredPitch, latestFilteredRoll);
        lastBLEUpdate = currentMillis;
    }
}