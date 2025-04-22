#pragma once

class DataProcessor {
public:
    static float calculateAccelerometerPitch(float ax, float ay, float az);
    static float calculateAccelerometerRoll(float ay, float az);
    static float calculateTotalInclination(float pitch, float roll);
    static float truncate(float value, int decimal_places);
};