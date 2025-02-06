// ******************* Imports e bibliotecas ******************* //
#include <Arduino.h>
#include <MPU9250.h>
#include "Config.h"
#include "KalmanFilter.h"
#include "SensorManager.h"
#include "DataProcessor.h"
#include "BluetoothLowEnergy.h"

// ******************* Instanciação de objetos e Variáveis Globais  ******************* //
MPU9250 mpu; // Instância do sensor MPU9250
MPU9250Setting mpu_settings; // Configurações do sensor MPU9250
TwoWire wire = Wire; // Interface I2C
SensorManager sensor(mpu, mpu_settings, Wire); // Gerenciador do sensor

KalmanFilter pitch_filter(
    Config::Kalman::PITCH_Q_ANGLE, // Valores definidos no namespace Config
    Config::Kalman::PITCH_Q_BIAS,
    Config::Kalman::PITCH_R_MEASURE
);

KalmanFilter roll_filter(
    Config::Kalman::ROLL_Q_ANGLE,
    Config::Kalman::ROLL_Q_BIAS,
    Config::Kalman::ROLL_R_MEASURE
);

// Utilizado para armazenar o tempo da última atualização
unsigned long last_update = 0;

// ******************* Funções Auxiliáres ******************* //

// Lidar com erros críticos no sensor
// Caso o sensor não seja encontrado no barramento I2C, piscar o led.
// Quando o sensor for encontrado, desligar o led.
void handleSensorError() {
    Serial.println("Erro crítico no sensor! Tentando reconectar...");
    bool mensagemImpressa = true;
    while (true) {
        // Pisca o LED para indicar erro
        digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        delay(100);
        if (sensor.initialize()) {
            digitalWrite(Config::LED_PIN, LOW);
            Serial.println("Sensor reconectado!");
            break;
        }
    }
}


// Inicializa os filtros de Kalman para os ângulos de pitch e roll
// Pega o primeiro dado dos sensores e inicializa os filtros com tais valores 
void initializeFilters() {
    const auto data = sensor.getData();
    pitch_filter.initialize(DataProcessor::calculateAccelerometerPitch(
        data.accel.x, data.accel.y, data.accel.z));
    roll_filter.initialize(DataProcessor::calculateAccelerometerRoll(
        data.accel.y, data.accel.z));
}

// ******************* Setup ******************* //
void setup() {
    Serial.begin(Config::SERIAL_BAUD_RATE); // Valores definidos no namespace Config
    pinMode(Config::LED_PIN, OUTPUT);

    if(!sensor.initialize()) {
        handleSensorError();
    }

    setupBLE(); // Inicializa o serviço BLE

    sensor.calibrateAccelGyro(); // Calibração do acelerômetro e giroscópio
    initializeFilters();
    last_update = millis();
}

// ******************* Loop principal ******************* //
void loop() {
    if(sensor.update()) { // Caso novos dados tenham sido adquiridos
        const unsigned long now = millis();
        const float delta_time = (now - last_update) / 1000.0f; // Tempo, em segundos, decorrido desde últia medição
        last_update = now;

        const auto data = sensor.getData(); // Objeto SensorData com valores lidos pelo sensor

        // Passa valores para DataProcessor calcular pitch e roll
        const float pitch_acc = DataProcessor::calculateAccelerometerPitch( 
            data.accel.x, data.accel.y, data.accel.z);

        const float roll_acc = DataProcessor::calculateAccelerometerRoll(
            data.accel.y, data.accel.z);

        // Filtra valores de pitch e roll com o Filtro de Kalman
        const float filtered_pitch = pitch_filter.update(
            pitch_acc, data.gyro.y, delta_time);

        const float filtered_roll = roll_filter.update(
            roll_acc, data.gyro.x, delta_time);

        // Calcula a inclinação total do sensor
        const float total_inclination = DataProcessor::calculateTotalInclination(
            filtered_pitch, filtered_roll);

        sendAngleValue(total_inclination); // Envia o valor da inclinação via BLE

        // Display dos valores em monitor serial para debbuging e análise
        Serial.printf(
            "Y:%.1f P:%.1f R:%.1f | FP:%.1f FR:%.1f | Inc:%.1f | Mag[%.1f,%.1f,%.1f]\n",
            data.yaw, data.pitch, data.roll,
            filtered_pitch, filtered_roll,
            total_inclination,
            data.mag.x, data.mag.y, data.mag.z
        );
    } 
}