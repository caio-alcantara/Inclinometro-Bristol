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
#include <SPI.h>
#include <SD.h>
#include "FFat.h"

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

// Preseça de cartão SD
bool sd_present = false;

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

void storeBatteryData(float batteryPercentage, float batteryVoltage, unsigned long timestamp, bool sd_present) {
    if (sd_present) {
        File batteryFile = SD.open("/battery.csv", FILE_APPEND);
        if (batteryFile) {
            batteryFile.print(batteryPercentage);
            batteryFile.print(",");
            batteryFile.print(batteryVoltage);
            batteryFile.print(",");
            batteryFile.println(timestamp);
            batteryFile.close();
        }
    }
}

void storeInclinationData(float total_inclination, float pitch_inclination, unsigned long timestamp, bool sd_present) {
    if (sd_present) {
        File pitchInclination = SD.open("/pitch_inclination.csv", FILE_APPEND);
        if (pitchInclination) {
            pitchInclination.print(pitch_inclination);
            pitchInclination.print(",");
            pitchInclination.println(timestamp);
            pitchInclination.close();
        }

        File totalInclinationFile = SD.open("/total_inclination.csv", FILE_APPEND);
        if (totalInclinationFile) {
            totalInclinationFile.print(total_inclination);
            totalInclinationFile.print(",");
            totalInclinationFile.println(timestamp);
            totalInclinationFile.close();
        }
    }
}

void setupSDCard() {
    SPI.begin(Config::SD_CLK_PIN, Config::SD_MISO_PIN, Config::SD_MOSI_PIN, Config::SD_CS_PIN);  
    
    if (!SD.begin(Config::SD_CS_PIN)) {
        Serial.println("Falha ao inicializar o cartão SD.");
    } else {
        Serial.println("Cartão SD inicializado com sucesso.");
        sd_present = true;

        // Configuração de arquivos de log

        File totalInclinationFile = SD.open("/total_inclination.csv", FILE_APPEND);
        if (totalInclinationFile) {
            if (totalInclinationFile.size() == 0) {
                totalInclinationFile.println("total_inclination,timestamp");
            }
            totalInclinationFile.close();
        }

        File batteryFile = SD.open("/battery.csv", FILE_APPEND);
        if (batteryFile) {
            if (batteryFile.size() == 0) {
                batteryFile.println("battery,voltage,timestamp");
            }
            batteryFile.close();
        }

        File pitchInclination = SD.open("/pitch_inclination.csv", FILE_APPEND);
        if (pitchInclination) {
            if (pitchInclination.size() == 0) {
                pitchInclination.println("pitch_inclination,timestamp");
            }
            pitchInclination.close();
        }
    }
}


// ******************* Setup ******************* //
void setup() {
    pauseNotifications = false; // Permite notificações BLE
    Serial.begin(Config::SERIAL_BAUD_RATE); // Valores definidos no namespace Config
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN); // Inicializa a comunicação I2C
    Wire.setClock(400000); // Define a velocidade do I2C para 100 kHz
    pinMode(Config::LED_PIN, OUTPUT);
    pinMode(Config::BLE_LED_PIN, OUTPUT);
    if(!sensor.initialize()) {
        handleSensorError();
    }


    if (!FFat.begin(FORMAT_FFAT_IF_FAILED)) {
        Serial.println("FFat Mount Failed even after formatting!");
        return;
    }

    setupBLE(); // Inicializa o serviço BLE

    sensor.calibrateAccelGyro(); // Calibração do acelerômetro e giroscópio
    initializeFilters();
    last_update = millis();

    // Inicializa o sensor MAX17043
    //if (!lipo.begin()) {
    //    handleBatteryError();
    //}

    Serial.println("MAX17043 iniciado com sucesso.");
    //lipo.quickStart();
    Serial.println("Quick Start executado no MAX17043.");

    lastOtaNotification = 0;
    lastSensorNotification = 0;

    // Inicializa o cartão SD
    //setupSDCard();
}

// ******************* Loop principal ******************* //
void loop() {
    // Verifica o modo de operação atual
    switch (MODE) {
        case NORMAL_MODE:
            // Modo normal de operação - processamento de sensores e notificações BLE
            if (deviceConnected) {
                digitalWrite(Config::BLE_LED_PIN, HIGH);
                
                // Comandos do protocolo OTA 
                if (sendMode) {
                    uint8_t fMode[] = {0xAA, FASTMODE};
                    registerOtaNotification();
                    pCharacteristicTX->setValue(fMode, 2);
                    pCharacteristicTX->notify();
                    delay(50); // Delay importante para estabilidade do BLE
                    sendMode = false;
                }
        
                if (sendSize) {
                    unsigned long x = FLASH.totalBytes();
                    unsigned long y = FLASH.usedBytes();
                    uint8_t fSize[] = {0xEF, (uint8_t) (x >> 16), (uint8_t) (x >> 8), (uint8_t) x, 
                                      (uint8_t) (y >> 16), (uint8_t) (y >> 8), (uint8_t) y};
                    pCharacteristicTX->setValue(fSize, 7);
                    pCharacteristicTX->notify();
                    delay(50); // Delay importante para estabilidade do BLE
                    sendSize = false;
                }
                
                // Processamento dos sensores apenas se não estiver pausado para OTA
                if (!pauseNotifications) {
                    unsigned long current_time = millis();
    
                    // Verifica a bateria a cada intervalo definido
                    if (current_time - last_battery_check >= Config::battery_check_interval) {
                        last_battery_check = current_time;
    
                        // Código de leitura da bateria aqui...
                        // float carga = lipo.getSOC();
                        // float tensao = lipo.getVoltage();
                        
                        // Envio dos dados da bateria
                        // sendBatteryPercentage(carga);
                    }
    
                    // Atualiza os sensores a cada intervalo definido
                    if (current_time - last_sensor_update >= Config::sensor_update_interval) {
                        last_sensor_update = current_time;
    
                        if (sensor.update()) {
                            const unsigned long now = millis();
                            const float delta_time = (now - last_update) / 1000.0f;
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
                            
                            // Envia dados via BLE com delays adequados entre notificações
                            Serial.print("Pitch: ");
                            Serial.println(filtered_pitch);
                            Serial.print(" Roll: ");
                            Serial.println(filtered_roll);
                            Serial.print(" Inclinação Total: ");
                            Serial.println(total_inclination);
                            sendAngleValue(filtered_roll);
                            sendPitchAndRoll(filtered_pitch - 6, filtered_roll);
                            //delay(10);
                            //sendPitchAndRoll(filtered_pitch, filtered_roll);
                        }
                    }
                }
            } else {
                digitalWrite(Config::BLE_LED_PIN, LOW);
                blinkBleLed();
            }
            break;
        
        case UPDATE_MODE:
            // Modo de recebimento do firmware - nesse modo não processa sensores
            if (request) {
                uint8_t rq[] = {0xF1, (uint8_t)((cur + 1) >> 8), (uint8_t)((cur + 1) & 0xFF)};
                pCharacteristicTX->setValue(rq, 3);
                pCharacteristicTX->notify();
                delay(50);  // Delay importante para estabilidade do BLE
                request = false;
            }
        
            if (cur + 1 == parts) { // received complete file
                uint8_t com[] = {0xF2, (uint8_t)((cur + 1) >> 8), (uint8_t)((cur + 1) & 0xFF)};
                pCharacteristicTX->setValue(com, 3);
                pCharacteristicTX->notify();
                delay(50);  // Delay importante para estabilidade do BLE
                MODE = OTA_MODE;
            }
        
            if (writeFile) {
                // Escreve os dados do firmware no arquivo
                if (!current) {
                    writeBinary(FLASH, "/update.bin", updater, writeLen);
                } else {
                    writeBinary(FLASH, "/update.bin", updater2, writeLen2);
                }
            }
            break;
        
        case OTA_MODE:
            // Modo de instalação do firmware
            if (writeFile) {
                // Finaliza a escrita do arquivo
                if (!current) {
                    writeBinary(FLASH, "/update.bin", updater, writeLen);
                } else {
                    writeBinary(FLASH, "/update.bin", updater2, writeLen2);
                }
            }
        
            // Verifica se o arquivo foi recebido completo
            if (rParts == tParts) {
                Serial.println("Firmware recebido completamente");
                delay(1000);
                updateFromFS(FLASH);  // Inicia a atualização
            } else {
                writeFile = true;
                Serial.println("Recebimento do firmware incompleto");
                Serial.print("Esperado: ");
                Serial.print(tParts);
                Serial.print(" Recebido: ");
                Serial.println(rParts);
                delay(2000);
                // Se o firmware estiver incompleto, volta ao modo normal
                MODE = NORMAL_MODE;
                pauseNotifications = false;
            }
            break;
    }
}