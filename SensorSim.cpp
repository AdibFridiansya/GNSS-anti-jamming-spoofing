#include "SensorSim.h"
#include <iostream>
#include <sstream>

SensorSim::SensorSim(const std::string& filepath) : filename(filepath) {}

SensorSim::~SensorSim() {
    if (file.is_open()) {
        file.close();
    }
}

bool SensorSim::initialize() {
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "[Error] Gagal membuka file simulasi: " << filename << std::endl;
        return false;
    }
    std::cout << "File simulasi berhasil dibuka: " << filename << std::endl;
    return true;
}

SimDataPacket SensorSim::readNextRow() {
    SimDataPacket packet = {};
    packet.isValid = false; 

    if (!file.is_open()) return packet;

    std::string line;
    
    while (std::getline(file, line)) {
        
        if (line.empty() || line[0] == '#' || line.find("time") != std::string::npos) {
            continue; 
        }

        std::stringstream ss(line);
        std::string token;

        try {
            // time, acc, gyr, mag, lat, lon, alt
            std::getline(ss, token, ','); packet.time = std::stod(token);
            std::getline(ss, token, ','); packet.accX = std::stod(token);
            std::getline(ss, token, ','); packet.accY = std::stod(token);
            std::getline(ss, token, ','); packet.accZ = std::stod(token);
            
            std::getline(ss, token, ','); packet.gyrX = std::stod(token);
            std::getline(ss, token, ','); packet.gyrY = std::stod(token);
            std::getline(ss, token, ','); packet.gyrZ = std::stod(token);
            
            std::getline(ss, token, ','); packet.magX = std::stod(token);
            std::getline(ss, token, ','); packet.magY = std::stod(token);
            std::getline(ss, token, ','); packet.magZ = std::stod(token);
            
            std::getline(ss, token, ','); packet.lat = std::stod(token);
            std::getline(ss, token, ','); packet.lon = std::stod(token);
            std::getline(ss, token, ','); packet.alt = std::stod(token);
            
            packet.isValid = true;
            return packet; 
            
        } catch (const std::exception& e) {
            std::cerr << "Melewati baris..." << std::endl;
            continue; 
        }
    }

    return packet; 
}