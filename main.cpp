#include <iostream>
#include <thread>
#include <chrono>

#include "IMUPreProcessing.h"

// ====================================================================
// THREE-WAY BRIDGING ARCHITECTURE: STM32 vs RASPI vs PC (SITL)
// ====================================================================
#if defined(USE_STM32_HARDWARE)
    // #include "HardwareSensorSTM32.h" // Modul sensor STM32 Anda nantinya
#elif defined(USE_RASPI_HARDWARE)
    #include "HardwareSensor.h"      // Berisi RTIMULib.h (Khusus Linux/I2C Raspi)
#else
    #include "SensorSim.h"           // Simulasi CSV (SITL PC)
#endif

// Konstanta untuk pengaturan loop rate simulasi (misal: 100 Hz -> 10 milidetik)
constexpr int SIMULATION_LOOP_DELAY_MS = 10;

int main() {
    std::cout << "=========================================" << std::endl;
    std::cout << "    SISTEM NAVIGASI INERSIAL - START     " << std::endl;
    std::cout << "    MODUL: IMU PREPROCESSING ONLY        " << std::endl;
    std::cout << "=========================================" << std::endl;

    // 1. Inisialisasi Modul IMU Preprocessing
    IMUPreProcessing imuProcessor;

    // 2. Konfigurasi Parameter Kalibrasi (KOREKSI PRESISI EIGEN)
    // Menggunakan tipe data float (f) sesuai definisi Vector3f & Matrix3f di header
    
    // Bias Statis Akselerometer & Giroskop
    Eigen::Vector3f staticAccBias(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f staticGyrBias(0.0f, 0.0f, 0.0f);
    imuProcessor.setBiasesStatic(staticAccBias, staticGyrBias);

    // Matriks Scale Factor & Cross-Coupling
    Eigen::Matrix3f accScaleCross = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f gyrScaleCross = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f gyrGDependent = Eigen::Matrix3f::Zero();
    imuProcessor.setScaleCrossMatrices(accScaleCross, gyrScaleCross, gyrGDependent);

    // 3. Inisialisasi Sensor Sesuai Platform
#if defined(USE_STM32_HARDWARE)
    std::cout << "[INFO] Mode STM32 Bare-Metal Aktif." << std::endl;
    // HardwareSensorSTM32 sensor;
    // sensor.initialize();
    
#elif defined(USE_RASPI_HARDWARE)
    std::cout << "[INFO] Mode RASPBERRY PI I2C Aktif." << std::endl;
    HardwareSensor sensor;
    if (!sensor.initialize()) {
        std::cerr << "[ERROR] Gagal inisialisasi RTIMULib via I2C!" << std::endl;
        return -1;
    }
    
#else
    std::cout << "[INFO] Mode SIMULASI (SITL) Aktif." << std::endl;
    const std::string csvFilename = "IMUsimulation.csv";
    SensorSim sensor(csvFilename);
    if (!sensor.initialize()) {
        std::cerr << "[ERROR] Gagal memuat file simulasi: " << csvFilename << std::endl;
        return -1;
    }
#endif

    std::cout << "[INFO] Memulai siklus pemrosesan data real-time...\n" << std::endl;

    // 4. Main Processing Loop (Terisolasi pada Prapemrosesan)
    while (true) {
        
        // A. Polimorfisme Pengambilan Data Raw
#if defined(USE_STM32_HARDWARE) || defined(USE_RASPI_HARDWARE)
        IMUDataPacket rawPacket = sensor.readData();
        if (!rawPacket.isDataValid) continue; // Abaikan jika pembacaan I2C/SPI gagal
#else
        SimDataPacket rawPacket = sensor.readNextRow();
        if (!rawPacket.isValid) {
            std::cout << "\n[INFO] Seluruh data simulasi selesai dibaca." << std::endl;
            break;
        }
#endif

        // B. Masukkan data mentah ke modul Preprocessing 
        // (Compiler akan otomatis memilih fungsi overloading yang tepat berdasarkan tipe paket)
        imuProcessor.inputRawData(rawPacket);

        // C. Ambil hasil olahan bersih (f dan w) dari Preprocessing
        ProcessedIMUData cleanIMU = imuProcessor.getProcessedData();

        // D. Validasi Output IMU Preprocessing (Pemantauan Terminal)
        if (cleanIMU.isValid) {
            std::cout << "Waktu     : " << cleanIMU.time << " s\n"
                      << "Clean Acc : [" << cleanIMU.acc.x() << ", " 
                      << cleanIMU.acc.y() << ", " << cleanIMU.acc.z() << "] m/s^2\n"
                      << "Clean Gyr : [" << cleanIMU.gyr.x() << ", " 
                      << cleanIMU.gyr.y() << ", " << cleanIMU.gyr.z() << "] rad/s\n"
                      << "---------------------------------------------------\n";
        }

        // E. Sinkronisasi (Menyerupai frekuensi perangkat keras asli)
        // Dilarang keras menidurkan thread (sleep) di STM32 karena akan memblokir RTOS/Superloop
#if !defined(USE_STM32_HARDWARE)
        std::this_thread::sleep_for(std::chrono::milliseconds(SIMULATION_LOOP_DELAY_MS));
#endif
    }

    std::cout << "=========================================" << std::endl;
    std::cout << "     PROGRAM BERHENTI DENGAN AMAN        " << std::endl;
    std::cout << "=========================================" << std::endl;

    return 0;
}