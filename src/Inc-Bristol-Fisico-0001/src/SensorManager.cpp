#include "SensorManager.h"
#include "Config.h"

// *********************** SensorManager *********************** //
// Feita para inicializar sensor, calibrar, atualizar dados e fornecer dados em estrutura adequada

// Construtor
SensorManager::SensorManager(MPU9250& sensor, MPU9250Setting& settings, TwoWire& wire) 
    : mpu(sensor), settings(settings), wire(wire) {}
    // Referencia ao objeto do sensor MPU9250, settings do sensor e interface I2C
    // Utilizei referências para evitar cópias desnecessárias de objetos

// Inicializar sensor (endereço I2C padrão é 0x68, mas pode ser alterado na placa do sensor)
bool SensorManager::initialize() {
    wire.begin();
    wire.setClock(100000); // Define a velocidade do I2C para 100 kHz
    return mpu.setup(Config::MPU_I2C_ADDRESS, settings, wire); // Retorno: true se o sensor foi inicializado com sucesso, false caso contrário
}

// calibração do acelerômetro e do giroscópio
// Remove erros sistemáticos (bias) do sensor
void SensorManager::calibrateAccelGyro() {
    mpu.calibrateAccelGyro();
}

// Atualiza os dados do sensor
bool SensorManager::update() { 
    return mpu.update(); 
}

// Retorna os dados do sensor em uma estrutura adequada (SensorData)
SensorManager::SensorData SensorManager::getData() const {
    return {
        {mpu.getAccX(), mpu.getAccY(), mpu.getAccZ()},
        {mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ()},
        {mpu.getMagX(), mpu.getMagY(), mpu.getMagZ()},
        mpu.getYaw(), mpu.getPitch(), mpu.getRoll()
    };
}