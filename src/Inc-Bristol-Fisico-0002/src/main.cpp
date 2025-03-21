// ******************* Imports e bibliotecas ******************* //
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include "Config.h"
#include "KalmanFilter.h"
#include "SensorManager.h"
#include "DataProcessor.h"
#include "BluetoothLowEnergy.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// ******************* Instanciação de objetos e Variáveis Globais  ******************* //
MPU9250 mpu; // Instância do sensor MPU9250
MPU9250Setting mpu_settings; // Configurações do sensor MPU9250
SensorManager sensor(mpu); // Gerenciador do sensor

SFE_MAX1704X lipo; // Instância do sensor de bateria

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
static unsigned long last_battery_check = 0;
static unsigned long last_sensor_update = 0;

// ******************* Funções Auxiliáres ******************* //

// Lidar com erros críticos no sensor
// Caso o sensor não seja encontrado no barramento I2C, piscar o led.
// Quando o sensor for encontrado, desligar o led.
void handleSensorError() {
    Serial.println("Erro crítico no sensor! Tentando reconectar...");
    bool mensagemImpressa = true;
    unsigned long previousMillis = 0;
    const unsigned long interval = 100;

    while (true) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            // Pisca o LED para indicar erro
            digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        }
        if (sensor.initialize()) {
            digitalWrite(Config::LED_PIN, LOW);
            Serial.println("Sensor reconectado!");
            break;
        }
    }
}

void handleBatteryError(){
    Serial.println("Erro crítico na bateria! Tentando reconectar...");
    bool mensagemImpressa = true;
    unsigned long previousMillis = 0;
    const unsigned long interval = 100;

    while (true) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            // Pisca o LED para indicar erro
            digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        }
        if (lipo.begin()) {
            digitalWrite(Config::LED_PIN, LOW);
            Serial.println("Bateria reconectada!");
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
    Wire.begin(21, 22); // Inicializa a comunicação I2C
    Wire.setClock(400000); // Define a velocidade do I2C para 100 kHz
    pinMode(Config::LED_PIN, OUTPUT);
    pinMode(Config::BLE_LED_PIN, OUTPUT);
    if(!sensor.initialize()) {
        handleSensorError();
    }

    setupBLE(); // Inicializa o serviço BLE

    sensor.calibrateAccelGyro(); // Calibração do acelerômetro e giroscópio
    initializeFilters();
    last_update = millis();

    // Inicializa o sensor MAX17043
    if (!lipo.begin()) {
        handleBatteryError();
    }

    Serial.println("MAX17043 iniciado com sucesso.");
    lipo.quickStart();
    Serial.println("Quick Start executado no MAX17043.");
}

// ******************* Loop principal ******************* //
void loop() {
    if (!deviceConnected) {
        blinkBleLed();
    } else {
        digitalWrite(Config::BLE_LED_PIN, HIGH);
        unsigned long current_time = millis();

        // Verifica a bateria a cada intervalo definido
        if (current_time - last_battery_check >= Config::battery_check_interval) {
            last_battery_check = current_time;

            float carga = lipo.getSOC();
            float tensao = lipo.getVoltage();

            Serial.print("Carga da bateria: ");
            Serial.print(carga, 2);
            Serial.println("%");
            
            Serial.print("Tensão da bateria: ");
            Serial.print(tensao, 2);
            Serial.println("V");

            sendBatteryPercentage(carga);
        }

        // Atualiza os sensores a cada intervalo definido
        if (current_time - last_sensor_update >= Config::sensor_update_interval) {
            last_sensor_update = current_time;

            if (sensor.update()) {
                const unsigned long now = millis();
                const float delta_time = (now - last_update) / 1000.0f; // Tempo, em segundos, decorrido desde última medição
                last_update = now;

                const auto data = sensor.getData();

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
                
                Serial.println("Dados do sensor:");
                Serial.printf("Inclinação total: %.2f\n", total_inclination);

                sendAngleValue(total_inclination); // Envia o valor da inclinação via BLE
                sendPitchAndRoll(filtered_pitch, filtered_roll); // Envia pitch e roll via BLE
            }
        }
    }

    
}
