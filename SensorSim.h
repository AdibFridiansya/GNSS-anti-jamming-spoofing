#pragma once

#include <string>
#include <fstream>

struct SimDataPacket {
    float time;
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float lat, lon, alt;
    bool isValid; 
};

class SensorSim {
private:
    std::ifstream file;
    std::string filename;

public:
    SensorSim(const std::string& filepath);
    ~SensorSim();
    bool initialize();
    SimDataPacket readNextRow();
};
