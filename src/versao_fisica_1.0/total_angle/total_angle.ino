// ******************* Imports e bibliotecas ******************* 
#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>
#include <math.h>

// *********************** Configurações ***********************
namespace Config {
    // TODO: Adicionar lógica para verificar os dois endereços (0x68 e 069)
    // Caso não consiga encontrar o dispositivo em um, tenta no outro. Se não achar em nenhum dos dois... problema
    constexpr uint8_t MPU_I2C_ADDRESS = 0x68;
    constexpr uint8_t LED_PIN = 2;
    constexpr unsigned long SERIAL_BAUD_RATE = 115200;
    
    namespace Kalman {
        /*
        Q_Angle: 
        Controla o quanto o filtro "confia" na previsão do giroscópio
        Valores menores: o filtro confia mais no giroscópio
        Valores maiores: o filtro confia menos no giroscópio

        Q_Bias:
        Controla o quanto o filtro ajusta o "erro sistemático" do giroscópio
        Valores menores: o filtro ajusta menos o bias
        Valores maiores: o filtro ajusta mais o bias

        R_Measuse:
        Controla o quanto o filtro "confia" no acelerômetro
        Valores menores: o filtro confia mais no acelerômetro
        Valores maiores: o filtro confia menos no acelerômetro
        */
        constexpr float PITCH_Q_ANGLE = 0.001f;
        constexpr float PITCH_Q_BIAS = 0.003f;
        constexpr float PITCH_R_MEASURE = 0.03f;
        
        // Separa se o PITCH (inclinação frontal, frente/trás) e o ROLL (Inclinação lateral, para os lados) porque eles podem possuir comportamentos diferentes

        constexpr float ROLL_Q_ANGLE = 0.001f;
        constexpr float ROLL_Q_BIAS = 0.003f;
        constexpr float ROLL_R_MEASURE = 0.03f;
    }
}

// *********************** KalmanFilter ***********************
// O Filtro de Kalman é um algoritmo matemático usado para estimar o estado de um sistema a partir de medições ruidosas
// Aqui, o Filtro de Kalman é utilizado para realizar a fusão entre os dados do acelerômetro e do giroscópio a fim de obter o ângulo 
// total do dispositivo. O filtro nos ajuda a diminuir ruidos e erros provenientes de vibrações
class KalmanFilter {
public:
    // Construtor
    KalmanFilter(float q_angle, float q_bias, float r_measure)
        : Q_angle(q_angle), Q_bias(q_bias), R_measure(r_measure) {} // Definidos em Config::Kalman::PITCH_Q_ANGLE...

    // Configura o estado inicial do filtro
    void initialize(float initial_angle) {
        angle = initial_angle;
        bias = 0.0f;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f; // Zera tudo para a primeira medição
    }

    // Atualiza o filtro com uma nova medição, mantendo uma malha fechada com feedback vindos dos sensores e possibilitando um cálculo sempre atualizado
    float update(float measurement, float rate, float delta_time) { // measurement: accel; rate: gyro; delta_time: tempo desde ultima medição
        /* As etapas seguem como:
        1. Predição: Atualiza o estado e a matriz de covariância com base no giroscópio
        2. Cálculo do Ganho de Kalman: Determina o peso da medição e da previsão
        3. Correção: Ajusta o estado com base na medição
        4. Atualização da Matriz de Covariância: Atualiza a incerteza da estimativa
        */ 
        predict(rate, delta_time);
        calculateKalmanGain();
        correct(measurement);
        updateCovarianceMatrix();
        return angle; // Angulo final estimado após a atualização
    }

private:
    const float Q_angle, Q_bias, R_measure;
    float angle = 0.0f; 
    float bias = 0.0f; 
    float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // Matriz de covariância do erro

    // Atualiza o estado e a matriz de covariância com base no giroscópio
    void predict(float rate, float delta_time) {
        // Angulo integra a taxa angular do giroscópio, já corrigida pelo bias
        angle += delta_time * (rate - bias);

        // Matriz de covariância atualiza a incerteza da estimativa
        P[0][0] += delta_time * (delta_time * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= delta_time * P[1][1];
        P[1][0] -= delta_time * P[1][1];
        P[1][1] += Q_bias * delta_time;
    }

    void calculateKalmanGain() {
        const float S = P[0][0] + R_measure;
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
    }

    void correct(float measurement) {
        const float innovation = measurement - angle;
        angle += K[0] * innovation;
        bias += K[1] * innovation;
    }

    void updateCovarianceMatrix() {
        const float P00_temp = P[0][0];
        const float P01_temp = P[0][1];
        
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
    }

    float K[2] = {0.0f, 0.0f};
};

// *********************** SensorManager ***********************
// Feita para inicializar sensor, calibrar, atualizar dados e fornecer dados em estrutura adequada
class SensorManager {
public:
    // Construtor
    SensorManager(MPU9250& sensor, MPU9250Setting& settings, TwoWire& wire) // Referencia ao objeto do sensor MPU9250, settings do sensor e interface I2C
        : mpu(sensor), settings(settings), wire(wire) {}                    // Utilizei referências para evitar cópias desnecessárias de objetos

    // Inicializar sensor (endereço I2C padrão é 0x68, mas pode ser alterado na placa do sensor)
    bool initialize() {
        wire.begin();
        return mpu.setup(Config::MPU_I2C_ADDRESS, settings, wire); // Retorno: true se o sensor foi inicializado com sucesso, false caso contrário
    }

    // calibração do acelerômetro e do giroscópio
    // Remove erros sistemáticos (bias) do sensor
    void calibrateAccelGyro() {
        Serial.println("Calibrando acelerômetro e giroscópio...");
        mpu.calibrateAccelGyro();
    }

    bool update() { return mpu.update(); }

    /* Define uma estrutura para armazenar os dados do sensor de forma organizada

    accel: Dados do acelerômetro (aceleração nos eixos X, Y, Z).
    gyro: Dados do giroscópio (velocidade angular nos eixos X, Y, Z).
    mag: Dados do magnetômetro (campo magnético nos eixos X, Y, Z).
    yaw, pitch, roll: Ângulos de orientação calculados pelo sensor.

    */
    struct SensorData {
        struct { float x, y, z; } accel, gyro, mag;
        float yaw, pitch, roll;
    };

    SensorData getData() const {
        return { // Retorno: Um objeto SensorData contendo todos os dados do sensor
            {mpu.getAccX(), mpu.getAccY(), mpu.getAccZ()},
            {mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ()},
            {mpu.getMagX(), mpu.getMagY(), mpu.getMagZ()},
            mpu.getYaw(), mpu.getPitch(), mpu.getRoll()
        };
    }

private:
    MPU9250& mpu;
    MPU9250Setting& settings;
    TwoWire& wire;
};

// *********************** DataProcessor ***********************
// Responsável por processar os dados brutos do sensor e realizar cálculos úteis,
// como a conversão de aceleração em ângulos de inclinação e o cálculo da inclinação total
class DataProcessor {
public:
    // Calcula o ângulo de inclinação frontal (pitch) com base nas acelerações medidas pelo acelerômetro
    static float calculateAccelerometerPitch(float ax, float ay, float az) {
        return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI; // Multiplica por 180.0f / PI para converter o resultado de radianos para graus
    }

    // Calcula o ângulo de inclinação lateral (roll) com base nas acelerações medidas pelo acelerômetro
    static float calculateAccelerometerRoll(float ay, float az) {
        return atan2(ay, az) * 180.0f / PI; // Multiplica por 180.0f / PI para converter o resultado de radianos para graus
    }


    static float calculateTotalInclination(float pitch, float roll) {
        float inclination = sqrt(pitch * pitch + roll * roll);
        if ((inclination < 15 && inclination > 0) || inclination > 85) {
          inclination -= 5; // Aqui, o -5 foi utilizado como valor descoberto empiricamente
                                                       // Basta colocar o sensor em posição de 90 graus e ver o quão distante a medição está de 90
                                                       // Neste caso, a 90 graus o sensor media 95. Dessa forma, subtraímos 5
        }

        if (inclination < 0) {
          inclination = 0;
        }
        
        inclination = truncate(inclination, 2); // Truncate arredonda para 2 casas decimais
        return (inclination > 95.0f) ? inclination - 90.0f : inclination; // Se a inclinação for maior que 95°, subtrai 90° para normalizar o valor.
                                                                          // Ou seja, o valor tende sempre a ser entre 0 e 90, independentemente da direção
    }

    // Arredondar valores a n casas decimais 
    static float truncate(float value, int decimal_places) {
        const float factor = pow(10, decimal_places);
        return (value > 0.0f) ? floor(value * factor) / factor : ceil(value * factor) / factor;
    }
};

// *********************** Variáveis Globais ***********************
MPU9250 mpu; //Objeto do sensor MPU9250
MPU9250Setting mpu_settings; // Objeto de configurações do sensor
TwoWire wire = Wire; // Interface I2C
SensorManager sensor(mpu, mpu_settings, wire);
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
unsigned long last_update = 0;

// *********************** Funções de Apoio ***********************
// Caso o sensor não seja encontrado no barramento I2C, piscar o led.
void handleSensorError() {
    Serial.println("Erro crítico no sensor!");
    while(true) {
        digitalWrite(Config::LED_PIN, !digitalRead(Config::LED_PIN));
        delay(100);
        if (sensor.initialize()) {
          digitalWrite(Config::LED_PIN, LOW); 
          break;
        }
        


    }
}

// Autiliar para inicializar os filtros de Kalman para accel e gyro
// Pega o primeiro dado dos sensores e inicializa os filtros com tais valores 
void initializeFilters() {
    const auto data = sensor.getData();
    pitch_filter.initialize(DataProcessor::calculateAccelerometerPitch(
        data.accel.x, data.accel.y, data.accel.z));
    roll_filter.initialize(DataProcessor::calculateAccelerometerRoll(
        data.accel.y, data.accel.z));
}

// *********************** Setup ***********************
void setup() {
    Serial.begin(Config::SERIAL_BAUD_RATE);
    pinMode(Config::LED_PIN, OUTPUT);

    if(!sensor.initialize()) {
        handleSensorError();
    }

    sensor.calibrateAccelGyro();
    initializeFilters();
    last_update = millis();
}

// *********************** Loop Principal ***********************
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

        const float total_inclination = DataProcessor::calculateTotalInclination(
            filtered_pitch, filtered_roll);

        // Display dos valores em monitor serial para debbuging 
        Serial.printf(
            "Y:%.1f P:%.1f R:%.1f | FP:%.1f FR:%.1f | Inc:%.1f | Mag[%.1f,%.1f,%.1f]\n",
            data.yaw, data.pitch, data.roll,
            filtered_pitch, filtered_roll,
            total_inclination,
            data.mag.x, data.mag.y, data.mag.z
        );
    }
}
