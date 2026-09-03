#pragma once

#include <cstdint>
// Bridging Architecture: STM32, Raspberry Pi, & PC (SITL) 
#ifdef USE_STM32_HARDWARE                                           // ARM-GCC Compiler (Hardware)
    #if defined(STM32H7xx) || defined(STM32H743xx)
        #include "stm32h7xx_hal.h"
    #elif defined(STM32F4xx) || defined(STM32F407xx)
        #include "stm32f4xx_hal.h"
    #elif defined(STM32F1xx) || defined(STM32F103xB)
        #include "stm32f1xx_hal.h"
    #elif defined(STM32G4xx)
        #include "stm32g4xx_hal.h"
    #else
        #error "STM32 architecture not recognized!"
    #endif                                 
#else                                                               // VSCode Compiler (SITL)
    #include <string>
    typedef void UART_HandleTypeDef;                                // Mocking struct STM32
    #ifndef USE_RASPI_HARDWARE
        #include <fstream>
    #endif
#endif

/* 
 (GNSSfix Type:) has 5 state, see section (32.17.17: UBX-NAV-PVT, Page 375).
 Documentation: u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
*/
enum class UBXFixType : uint8_t {
    NO_FIX = 0,
    DEAD_RECKONING = 1, 
    FIX_2D = 2,
    FIX_3D = 3,         
    GNSS_AND_DR = 4,    
    TIME_ONLY = 5
};

struct GNSSData {
    uint32_t timestamp_ms;                  // iTOW (ms)
    double latitude;        
    double longitude;       
    float altitude_msl;                     // (m)
    float velocity_N;                       // (m/s)
    float velocity_E;                       // (m/s)
    float velocity_D;                       // (m/s) 
    
    // Adaptive EKF (Optional)
    float h_acc;                            // Horizontal est.(m)
    float v_acc;                            // Vertical est.(m)
    float s_acc;                            // Velocity est.(m/s)
    
    UBXFixType fix_type;                    // Signal status (UBXFixType)
    uint8_t satellites;                     // Number of Satellite
    bool is_updated;         
};

class GNSSreader {
private:
    // Read Serial Packet Status (Non-Blocking), see Message Structure (Page 375).
    enum class ParserState {
        WAIT_SYNC_1,                        // Header 0xB5 (Mu)
        WAIT_SYNC_2,                        // Header 0x62 (b)
        GET_CLASS,                          // Class ID (0x01 - NAV)
        GET_ID,                             // Message ID (0x07 - PVT)
        GET_LENGTH_1,                       // LSB
        GET_LENGTH_2,                       // MSB
        GET_PAYLOAD,                        // Buffer
        GET_CHECKSUM_A, 
        GET_CHECKSUM_B  
    };

    GNSSData latest_data;

    // Hybrid Mode Control
    #if defined(USE_STM32_HARDWARE)
        UART_HandleTypeDef* huart_gnss;         // STM32
    #elif defined(USE_RASPI_HARDWARE)
        int serial_fd;                          // Raspberry Pi
    #else
        std::ifstream csv_file;                 // PC
    #endif
        
    // FSM State Memory
    ParserState state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_length;
    uint16_t payload_counter;
    // Fletcher's Checksum (RFC 1145)
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t rx_ck_a;                        
    // UBX-NAV-PVT has 92 Length (Bytes), padding aman ke 100
    uint8_t payload_buffer[100]; 

    void processByte(uint8_t byte);                         
    void calculateChecksum(uint8_t byte);                   
    void extractPVTPayload();                               

public:
    GNSSreader();
    ~GNSSreader();

#if defined(USE_STM32_HARDWARE)
    bool initHardware(UART_HandleTypeDef* huart_ptr); 
#elif defined(USE_RASPI_HARDWARE)
    bool initRaspi(const std::string& port_name, int baudrate);
#else
    bool initSimulation(const std::string& csv_filepath);
#endif

    bool readData(); 
    GNSSData getLatestData() const;
};