#ifndef __SD_H
#define __SD_H

#include "stm32f7xx_hal.h" // For HAL types and SPI/GPIO
#include "main.h"          // For GPIO pin definitions if they are there (e.g. SD_CS_Pin, SD_CS_GPIO_Port)
#include <stdint.h>
#include <stdbool.h>

// Define CS Pin and Port if not already defined in main.h specific to SD
// It's better if these are defined in main.h or a dedicated pin_config.h
// For this example, we'll assume they might be directly used in SD.c
// Or, ensure MX_GPIO_Init in main.c configures these pins.
#define SD_CS_GPIO_Port GPIOB
#define SD_CS_Pin       GPIO_PIN_12

// SD Card Commands (simplified set for basic operations)
#define CMD0   (0)         // GO_IDLE_STATE
#define CMD8   (8)         // SEND_IF_COND
#define CMD16  (16)        // SET_BLOCKLEN
#define CMD17  (17)        // READ_SINGLE_BLOCK
#define CMD24  (24)        // WRITE_SINGLE_BLOCK
#define CMD55  (55)        // APP_CMD (prefix for ACMDs)
#define CMD58  (58)        // READ_OCR
#define ACMD41 (41)        // SD_SEND_OP_COND (application-specific command)

// SD Card Response Types (simplified)
#define R1_IDLE_STATE          (1 << 0)
#define R1_ERASE_RESET         (1 << 1)
#define R1_ILLEGAL_COMMAND     (1 << 2)
#define R1_COM_CRC_ERROR       (1 << 3)
#define R1_ERASE_SEQUENCE_ERROR (1 << 4)
#define R1_ADDRESS_ERROR       (1 << 5)
#define R1_PARAMETER_ERROR     (1 << 6)

// Card type flags
#define SD_CARD_TYPE_SD1    1
#define SD_CARD_TYPE_SD2    2
#define SD_CARD_TYPE_SDHC   3 // SDHC or SDXC

typedef struct {
    SPI_HandleTypeDef *hspi; // Pointer to the SPI handle
    uint8_t card_type;       // SD_CARD_TYPE_SD1, SD_CARD_TYPE_SD2, SD_CARD_TYPE_SDHC
    bool initialized;
} SD_CardInfo_t;

// Data structure for logging
// Adjust types and add/remove members as needed for your specific data
typedef struct {
    uint32_t timestamp_ms;         // Loop start time or other timestamp

    float lsm_acc_x_mg;            // LSM6DSO Accel X (mg)
    float lsm_acc_y_mg;            // LSM6DSO Accel Y (mg)
    float lsm_acc_z_mg;            // LSM6DSO Accel Z (mg)

    float lsm_gyro_x_mdps;         // LSM6DSO Gyro X (mdps)
    float lsm_gyro_y_mdps;         // LSM6DSO Gyro Y (mdps)
    float lsm_gyro_z_mdps;         // LSM6DSO Gyro Z (mdps)

    float bmp_pres_pa;             // BMP390 Pressure (Pa)
    float bmp_alt_m;               // BMP390 Altitude (m)

    float adxl_hi_g_x;             // ADXL375 High-G X (g)
    float adxl_hi_g_y;             // ADXL375 High-G Y (g)
    float adxl_hi_g_z;             // ADXL375 High-G Z (g)

    float q0, q1, q2, q3;          // Madgwick Quaternions

    float kf_altitude_m;           // Kalman Filter Altitude (m)
    float kf_velocity_mps;         // Kalman Filter Velocity (m/s)

    uint32_t loop_exec_time_ms;    // Loop execution time
    uint8_t flight_phase;          // flight_fsm_e as uint8_t
    // Add other variables from your print statements if needed
    // For example, raw sensor values, data ready flags, etc.
} FlightData_LogEntry_t;

// Function Prototypes
void SD_SPI_Init(SPI_HandleTypeDef *hspi_instance); // To associate the SPI handle
uint8_t SD_Init(void);
uint8_t SD_Read_Block(uint32_t block_addr, uint8_t *data_buffer);
uint8_t SD_Write_Block(uint32_t block_addr, const uint8_t *data_buffer);

// Data Logging Functions
void SD_Log_Start(uint32_t start_block_address); // Initialize logging to a start block
void SD_Log_Data(const FlightData_LogEntry_t *log_entry); // Log one entry
void SD_Log_FlushBuffer(void); // Force write of any pending data in buffer
void SD_Log_Stop(void); // Finalize logging (e.g., flush buffer)

// Test function
void SD_Run_Test_Write(void);

#endif /* __SD_H */
