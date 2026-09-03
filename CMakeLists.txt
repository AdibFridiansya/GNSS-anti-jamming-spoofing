cmake_minimum_required(VERSION 3.10)
project(SistemNavigasi)

# Wajib menggunakan C++14 atau lebih tinggi untuk fitur multithreading
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Cari library thread bawaan dari sistem operasi
find_package(Threads REQUIRED)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/eigen-5.0.1")

# 1. Daftarkan semua file .cpp buatan Anda di sini untuk dijadikan satu aplikasi
add_executable(tes_navigasi 
    main.cpp 
    SensorSim.cpp 
    HardwareSensor.cpp 
    IMUPreProcessing.cpp 
    EKF.cpp
)

# 2. PENGATURAN RTIMULib2
add_subdirectory(RTIMULib2/RTIMULib)

# Beri tahu kompilator di mana letak file header (RTIMULib.h)
target_include_directories(tes_navigasi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/RTIMULib2/RTIMULib)

# 3. PROSES LINKING
target_link_libraries(tes_navigasi PRIVATE RTIMULib Threads::Threads)