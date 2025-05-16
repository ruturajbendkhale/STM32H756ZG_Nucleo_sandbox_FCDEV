/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include <stdio.h>
#include "lsm6dso.h" // Re-enable LSM6DSO include
#include "driver_bmp390.h" // New BMP390 driver header
#include <math.h>          // For powf in altitude calculation
#include <stdarg.h>        // For vsnprintf in debug print
#include "lsm6dso_reg.h" // For LSM6DSO_WHO_AM_I
#include "adxl375.h"     // Add ADXL375 support
#include <stdbool.h>
#include "Madgwick_filter.h" // Include Madgwick filter
#include "kalman_filter.h" // Include Kalman filter
#include "flight_phases.h" // Include for flight state machine
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP390_I2C_ADDRESS_LOW (0x76 << 1) // Default I2C address for BMP390 when SDO is low
#define BMP390_I2C_ADDRESS_HIGH (0x77 << 1) // I2C address for BMP390 when SDO is high
#define TARGET_LOOP_PERIOD_MS 10 // For 100Hz sampling frequency (1000ms / 100Hz = 10ms)

// Define these to print specific sensor data. Comment out to disable.
#define PRINT_LSM6DSO_ACCEL_DATA
#define PRINT_LSM6DSO_GYRO_DATA
#define PRINT_BMP390_DATA
#define PRINT_ADXL375_DATA
#define PRINT_QUATERNION_DATA
#define PRINT_LOOP_EXEC_TIME
#define PRINT_KALMAN_DATA // New define for Kalman filter output
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static stmdev_ctx_t dev_ctx; // Re-enable LSM6DSO context

bmp390_handle_t bmp390_handle;
char uart_buffer[256]; // Increased buffer size
float sea_level_pressure_hpa; // Store as hPa for altitude calculations

// LSM6DSO Offsets (mg for accelerometer, mdps for gyroscope)
static float lsm6dso_accel_offset_mg[3] = {0.0f, 0.0f, 0.0f};
static float lsm6dso_gyro_offset_mdps[3] = {0.0f, 0.0f, 0.0f};

// Flight State Machine and Settings
static flight_fsm_t flight_state_machine;
static control_settings_t control_settings;

#define LSM6DSO_I2C_ADD_L 0x6A  // Standard I2C address for LSM6DSO (0x6A when SDO/SA0 is connected to GND)
#define LSM6DSO_I2C_ADD_H 0x6B  // Alternative I2C address for LSM6DSO (0x6B when SDO/SA0 is connected to VDD)

// Variables for LSM6DSO
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static uint8_t whoamI_lsm, rst_lsm;

static float latest_bmp_temp_c = 0.0f; // Stores the 1Hz temperature reading
static uint16_t bmp_temp_read_counter = 0;
const uint16_t BMP_TEMP_READ_INTERVAL = 100; // For 1Hz update (100 loops * 10ms/loop = 1000ms)

// Counter for scheduling BMP390 reads every 20ms (2 loop cycles)
static uint8_t bmp390_read_scheduler = 0;

// Persistent storage for last good BMP390 pressure and altitude
static float persistent_bmp_pres_pa = 0.0f;
static float persistent_bmp_alt_m = 0.0f;

// Kalman Filter instance
static KalmanFilter kf_altitude_velocity;

// LSM6DSO functions re-enabled and corrected
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  // Note: LSM6DSO_I2C_ADD_L is the 7-bit address. HAL functions expect the 8-bit address (7-bit shifted left).
  if (HAL_I2C_Mem_Write((I2C_HandleTypeDef*)handle, (LSM6DSO_I2C_ADD_L << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000) == HAL_OK) {
    return 0;
  }
  return -1; // Return non-zero for error
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  // Note: LSM6DSO_I2C_ADD_L is the 7-bit address. HAL functions expect the 8-bit address (7-bit shifted left).
  if (HAL_I2C_Mem_Read((I2C_HandleTypeDef*)handle, (LSM6DSO_I2C_ADD_L << 1), reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) == HAL_OK) {
    return 0;
  }
  return -1; // Return non-zero for error
}

// Reference sea level pressure (standard value is 1013.25 hPa = 1.01325 bar)
// float sea_level_pressure_bar = 1.01325f; // Old, replaced by sea_level_pressure_hpa

// Function to calibrate sea level pressure based on current altitude
// void calibrate_sea_level_pressure(float current_pressure_bar, float known_altitude_meters) { // Old
// Use the barometric formula to calculate the sea level pressure
//  sea_level_pressure_bar = current_pressure_bar / 
//                          powf((1.0f - (known_altitude_meters * 0.0000225577f)), 5.255877f);
//}

// Function to calculate altitude based on pressure and calibrated sea level pressure
//float calculate_altitude(float pressure_bar) { // Old
// Convert bar to Pa
//  float pressure_pa = pressure_bar * 100000.0f;
  
// International barometric formula (accurate for altitudes < 11km)
//  return 44330.0f * (1.0f - powf(pressure_pa / (sea_level_pressure_bar * 100000.0f), 0.1903f));
//}

// At the top of your file, declare a variable for LED blink timing
static uint32_t led3_last_toggle_time = 0; // Last time LED3 was toggled
const uint32_t led3_blink_interval = 200; // Blink interval in milliseconds

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

// Wrapper functions for BMP390 driver
uint8_t bmp390_i2c_interface_init(void) {
  // MX_I2C1_Init() is called before this, so I2C hardware is already initialized.
  // This function can be a no-op or ensure I2C1 is ready.
  return 0; // Success
}

uint8_t bmp390_i2c_interface_deinit(void) {
  // Optional: HAL_I2C_DeInit(&hi2c1); if necessary for power down
  return 0; // Success
}

uint8_t bmp390_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    // The driver's a_bmp390_iic_spi_read calls this with addr = handle->iic_addr
    if (HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY) == HAL_OK) {
        return 0; // Success
    }
    return 1; // Failure
}

uint8_t bmp390_i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    // The driver's a_bmp390_iic_spi_write calls this in a loop with len=1 for multi-byte writes.
    // So, this function will effectively be called to write one byte at a time.
    if (HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY) == HAL_OK) {
        return 0; // Success
    }
    return 1; // Failure
}

void bmp390_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

// Dummy SPI functions to satisfy driver checks when using I2C
uint8_t bmp390_spi_interface_init(void) {
  // This won't be called if I2C interface is selected
  return 0; // Success
}

uint8_t bmp390_spi_interface_deinit(void) {
  // This won't be called if I2C interface is selected
  return 0; // Success
}

uint8_t bmp390_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
  // This won't be called if I2C interface is selected
  (void)reg; // Suppress unused parameter warning
  (void)buf; // Suppress unused parameter warning
  (void)len; // Suppress unused parameter warning
  return 1; // Simulate failure if somehow called
}

uint8_t bmp390_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
  // This won't be called if I2C interface is selected
  (void)reg; // Suppress unused parameter warning
  (void)buf; // Suppress unused parameter warning
  (void)len; // Suppress unused parameter warning
  return 1; // Simulate failure if somehow called
}

void bmp390_debug_print(const char *const fmt, ...) {
    char dbg_buffer[128]; // Buffer for debug messages
    va_list args;
    va_start(args, fmt);
    vsnprintf(dbg_buffer, sizeof(dbg_buffer), fmt, args);
    va_end(args);
    // Direct transmit is simpler if buffer is managed carefully
    // HAL_UART_Transmit(&huart3, (uint8_t*)"BMP390_DBG: ", 12, HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buffer, strlen(dbg_buffer), HAL_MAX_DELAY);
    (void)dbg_buffer; // Suppress unused variable warning if UART lines are commented
}


// Altitude calculation functions
// pressure_hpa: current measured pressure in hPa
// known_altitude_meters: current known altitude in meters
void calibrate_sea_level_pressure_hpa(float current_pressure_hpa, float known_altitude_meters) {
  sea_level_pressure_hpa = current_pressure_hpa / powf((1.0f - (known_altitude_meters * 0.0000225577f)), 5.255877f);
}

// pressure_hpa: current measured pressure in hPa
// returns altitude in meters
float calculate_altitude_hpa(float pressure_hpa) {
  if (sea_level_pressure_hpa <= 0) return 0.0f; // Avoid division by zero or log of non-positive
  return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_pressure_hpa, 0.1903f));
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Helper functions to convert LSM6DSO raw data to physical units
// These are added here to resolve linker errors if not found in the ST driver files linked.
// Ensure these sensitivities match the configured Full-Scale settings for the sensor.

/**
  * @brief  Converts raw accelerometer data from LSM6DSO to mg.
  * @param  lsb Raw data from the sensor.
  * @retval Acceleration in mg.
  */
static float lsm6dso_from_fs16g_to_mg(int16_t lsb)
{
  // Apply a 2x correction factor - the sensor is reporting ~half the expected values
  return (float)lsb * 0.488f * 2.0f; // Sensitivity for +/-16g full scale with 2x correction
}

/**
  * @brief  Converts raw gyroscope data from LSM6DSO to mdps.
  * @param  lsb Raw data from the sensor.
  * @retval Angular rate in mdps.
  */
static float lsm6dso_from_fs250dps_to_mdps(int16_t lsb)
{
  return (float)lsb * 8.75f; // Sensitivity for +/-250dps full scale
}

/**
  * @brief  Converts raw accelerometer data from LSM6DSO to mg.
  * @param  lsb Raw data from the sensor.
  * @retval Acceleration in mg.
  */
static float lsm6dso_from_fs32g_to_mg(int16_t lsb)
{
  return (float)lsb * 0.976f; // Sensitivity for +/-32g full scale (0.976 mg/LSB)
}

/**
  * @brief  Converts raw gyroscope data from LSM6DSO to mdps.
  * @param  lsb Raw data from the sensor.
  * @retval Angular rate in mdps.
  */
static float lsm6dso_from_fs2000dps_to_mdps(int16_t lsb)
{
  return (float)lsb * 70.0f; // Sensitivity for +/-2000dps full scale (70 mdps/LSB)
}

// Function to calibrate sea level pressure based on current altitude
// void calibrate_sea_level_pressure(float current_pressure_bar, float known_altitude_meters) { // Old
// Use the barometric formula to calculate the sea level pressure
//  sea_level_pressure_bar = current_pressure_bar / 
//                          powf((1.0f - (known_altitude_meters * 0.0000225577f)), 5.255877f);
//}

// Function to calculate altitude based on pressure and calibrated sea level pressure
//float calculate_altitude(float pressure_bar) { // Old
// Convert bar to Pa
//  float pressure_pa = pressure_bar * 100000.0f;
  
// International barometric formula (accurate for altitudes < 11km)
//  return 44330.0f * (1.0f - powf(pressure_pa / (sea_level_pressure_bar * 100000.0f), 0.1903f));
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize I2C (already called by HAL_Init system, but good to ensure)
  // MX_I2C1_Init(); // Called above in peripheral init sequence

  // char uart_buffer[100]; // Moved global and resized
  
  sprintf(uart_buffer, "System Initialized. UART OK.\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  
  // Scan the I2C bus first to see what devices are present
  sprintf(uart_buffer, "Scanning I2C bus for devices...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  
  uint8_t i2c_devices_found = 0;
  for (uint8_t i = 1; i < 128; i++) {
    if (i < 0x08 || i > 0x77) continue;

    HAL_StatusTypeDef i2c_result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 10); // Use 2 trials, 10ms timeout
    
    if (i2c_result == HAL_OK) {
      i2c_devices_found++;
      sprintf(uart_buffer, "Found I2C device at address: 0x%02X\r\n", i);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
      if (i == (BMP390_I2C_ADDRESS_LOW >> 1) || i == (BMP390_I2C_ADDRESS_HIGH >> 1)) {
        sprintf(uart_buffer, "  --> This could be a BMP390 sensor!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      }
    }
  }
  
  if (i2c_devices_found == 0) {
    sprintf(uart_buffer, "No I2C devices found! Check wiring/pull-ups.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  } else {
    sprintf(uart_buffer, "Found %d I2C devices in total.\r\n", i2c_devices_found);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  }

  // Initialize LSM6DSO
  sprintf(uart_buffer, "Initializing LSM6DSO...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  // Check device ID
  lsm6dso_device_id_get(&dev_ctx, &whoamI_lsm);
  if (whoamI_lsm == LSM6DSO_WHO_AM_I) {
    sprintf(uart_buffer, "LSM6DSO WHO_AM_I is OK: 0x%02X\r\n", whoamI_lsm);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  } else {
    sprintf(uart_buffer, "LSM6DSO WHO_AM_I FAILED! Expected 0x%02X, got 0x%02X\r\n", LSM6DSO_WHO_AM_I, whoamI_lsm);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    // Error_Handler(); // Decide if this is fatal
  }

  // Restore default configuration
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&dev_ctx, &rst_lsm);
  } while (rst_lsm);

  // Enable Block Data Update
  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  // Set Output Data Rate for Accelerometer and Gyroscope
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_104Hz); // 104 Hz ODR for Accelerometer
  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_104Hz); // 104 Hz ODR for Gyroscope

  // Set Full Scale for Accelerometer and Gyroscope
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_16g);    // +/- 16g Full Scale for Accelerometer
  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps); // +/- 2000 dps Full Scale for Gyroscope

  // Make sure xl_fs_mode is set to 0 to ensure 16g works correctly
  uint8_t ctrl8_xl_val = 0;
  lsm6dso_read_reg(&dev_ctx, LSM6DSO_CTRL8_XL, &ctrl8_xl_val, 1);
  // Clear the xl_fs_mode bit (bit 1) to ensure proper 16g operation
  ctrl8_xl_val &= ~(1 << 1);
  lsm6dso_write_reg(&dev_ctx, LSM6DSO_CTRL8_XL, &ctrl8_xl_val, 1);
  
  // ---- START DEBUG: Read back CTRL1_XL and CTRL8_XL ----
  uint8_t ctrl1_xl_val = 0;
  if (lsm6dso_read_reg(&dev_ctx, LSM6DSO_CTRL1_XL, &ctrl1_xl_val, 1) == 0) {
    sprintf(uart_buffer, "LSM6DSO CTRL1_XL after set: 0x%02X\r\n", ctrl1_xl_val);
  } else {
    sprintf(uart_buffer, "LSM6DSO Failed to read CTRL1_XL\r\n");
  }
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  
  lsm6dso_read_reg(&dev_ctx, LSM6DSO_CTRL8_XL, &ctrl8_xl_val, 1);
  sprintf(uart_buffer, "LSM6DSO CTRL8_XL after set: 0x%02X\r\n", ctrl8_xl_val);
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  // ---- END DEBUG ----

  sprintf(uart_buffer, "LSM6DSO Initialized and Configured (XL:104Hz/16g, GY:104Hz/2000dps).\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  // Calibrate LSM6DSO Accelerometer and Gyroscope
  sprintf(uart_buffer, "Starting LSM6DSO calibration... Keep device still and flat for a few seconds.\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  const int lsm6dso_cal_samples = 100;
  float temp_accel_sum_mg[3] = {0.0f, 0.0f, 0.0f};
  float temp_gyro_sum_mdps[3] = {0.0f, 0.0f, 0.0f};
  int16_t raw_acc[3], raw_gyro[3];

  for (int i = 0; i < lsm6dso_cal_samples; i++) {
    uint8_t reg_lsm_cal;
    do {
        lsm6dso_status_reg_get(&dev_ctx, &reg_lsm_cal);
    } while (!((reg_lsm_cal & 0x01) && (reg_lsm_cal & 0x02))); // Wait for both Accel (XLDA) and Gyro (GDA) data ready

    lsm6dso_acceleration_raw_get(&dev_ctx, raw_acc);
    temp_accel_sum_mg[0] += lsm6dso_from_fs16g_to_mg(raw_acc[0]);
    temp_accel_sum_mg[1] += lsm6dso_from_fs16g_to_mg(raw_acc[1]);
    temp_accel_sum_mg[2] += lsm6dso_from_fs16g_to_mg(raw_acc[2]);

    lsm6dso_angular_rate_raw_get(&dev_ctx, raw_gyro);
    temp_gyro_sum_mdps[0] += lsm6dso_from_fs2000dps_to_mdps(raw_gyro[0]);
    temp_gyro_sum_mdps[1] += lsm6dso_from_fs2000dps_to_mdps(raw_gyro[1]);
    temp_gyro_sum_mdps[2] += lsm6dso_from_fs2000dps_to_mdps(raw_gyro[2]);

    HAL_Delay(10); // Delay between samples (LSM6DSO ODR is ~104Hz, so new data every ~9.6ms)
  }

  lsm6dso_accel_offset_mg[0] = temp_accel_sum_mg[0] / lsm6dso_cal_samples;
  lsm6dso_accel_offset_mg[1] = temp_accel_sum_mg[1] / lsm6dso_cal_samples;
  lsm6dso_accel_offset_mg[2] = (temp_accel_sum_mg[2] / lsm6dso_cal_samples) - 1000.0f; // Assuming Z-axis is UP, expecting +1g (1000mg)

  lsm6dso_gyro_offset_mdps[0] = temp_gyro_sum_mdps[0] / lsm6dso_cal_samples;
  lsm6dso_gyro_offset_mdps[1] = temp_gyro_sum_mdps[1] / lsm6dso_cal_samples;
  lsm6dso_gyro_offset_mdps[2] = temp_gyro_sum_mdps[2] / lsm6dso_cal_samples;

  sprintf(uart_buffer, "LSM6DSO Calibration Complete.\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  sprintf(uart_buffer, "  Accel Offsets (mg): X: %.2f, Y: %.2f, Z: %.2f\r\n", 
          lsm6dso_accel_offset_mg[0], lsm6dso_accel_offset_mg[1], lsm6dso_accel_offset_mg[2]);
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  sprintf(uart_buffer, "  Gyro Offsets (mdps): X: %.2f, Y: %.2f, Z: %.2f\r\n", 
          lsm6dso_gyro_offset_mdps[0], lsm6dso_gyro_offset_mdps[1], lsm6dso_gyro_offset_mdps[2]);
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  // Initialize BMP390 using the new driver
  DRIVER_BMP390_LINK_INIT(&bmp390_handle, bmp390_handle_t);
  DRIVER_BMP390_LINK_IIC_INIT(&bmp390_handle, bmp390_i2c_interface_init);
  DRIVER_BMP390_LINK_IIC_DEINIT(&bmp390_handle, bmp390_i2c_interface_deinit);
  DRIVER_BMP390_LINK_IIC_READ(&bmp390_handle, bmp390_i2c_read);
  DRIVER_BMP390_LINK_IIC_WRITE(&bmp390_handle, bmp390_i2c_write);
  DRIVER_BMP390_LINK_DELAY_MS(&bmp390_handle, bmp390_delay_ms);
  DRIVER_BMP390_LINK_DEBUG_PRINT(&bmp390_handle, bmp390_debug_print);

  // Link dummy SPI functions as well, even if not used, to satisfy driver checks
  DRIVER_BMP390_LINK_SPI_INIT(&bmp390_handle, bmp390_spi_interface_init);
  DRIVER_BMP390_LINK_SPI_DEINIT(&bmp390_handle, bmp390_spi_interface_deinit);
  DRIVER_BMP390_LINK_SPI_READ(&bmp390_handle, bmp390_spi_read);
  DRIVER_BMP390_LINK_SPI_WRITE(&bmp390_handle, bmp390_spi_write);

  bmp390_set_interface(&bmp390_handle, BMP390_INTERFACE_IIC);
  // IMPORTANT: Set the correct I2C address based on your SDO/AD0 pin connection
  bmp390_set_addr_pin(&bmp390_handle, BMP390_ADDRESS_ADO_HIGH); // Corrected: Use 0x77 as detected by scan

  sprintf(uart_buffer, "Initializing BMP390 (new driver)...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  if (bmp390_init(&bmp390_handle) != 0) {
      sprintf(uart_buffer, "BMP390 new driver initialization FAILED!\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      Error_Handler();
  } else {
      sprintf(uart_buffer, "BMP390 new driver initialized successfully!\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

      // Configure sensor settings
      sprintf(uart_buffer, "Configuring BMP390...\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

      if (bmp390_set_pressure_oversampling(&bmp390_handle, BMP390_OVERSAMPLING_x8) != 0) {
          sprintf(uart_buffer, "BMP390: Failed to set pressure oversampling\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      }
      if (bmp390_set_temperature_oversampling(&bmp390_handle, BMP390_OVERSAMPLING_x1) != 0) {
          sprintf(uart_buffer, "BMP390: Failed to set temperature oversampling\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      }
      if (bmp390_set_odr(&bmp390_handle, BMP390_ODR_25_HZ) != 0) { // Revert to 25 Hz ODR
          sprintf(uart_buffer, "BMP390: Failed to set ODR to 25Hz\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      }
      if (bmp390_set_pressure(&bmp390_handle, BMP390_BOOL_TRUE) != 0) { // Enable pressure
          sprintf(uart_buffer, "BMP390: Failed to enable pressure measurement\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      }
      if (bmp390_set_temperature(&bmp390_handle, BMP390_BOOL_TRUE) != 0) { // Enable temperature
          sprintf(uart_buffer, "BMP390: Failed to enable temperature measurement\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      }
      if (bmp390_set_mode(&bmp390_handle, BMP390_MODE_NORMAL_MODE) != 0) {
          sprintf(uart_buffer, "BMP390: Failed to set normal mode!\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY); Error_Handler();
      } else {
          sprintf(uart_buffer, "BMP390 configured for Normal Mode (P_OSR_x8, T_OSR_x1, ODR_25Hz).\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      }
  }

  HAL_Delay(200); // Wait for sensor to stabilize and take first readings after mode set

  float initial_pressure_pa_sum = 0;
  float initial_temperature_c_sum = 0;
  int valid_calibration_readings = 0;
  uint32_t cal_raw_p, cal_raw_t;
  float cal_p_pa, cal_t_c; // Pressure in Pa, Temp in Celsius

  sprintf(uart_buffer, "Calibrating BMP390 for altitude (takes a few readings)...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  for (int i = 0; i < 5; i++) { // Take 5 readings for averaging
      if (bmp390_read_temperature_pressure(&bmp390_handle, &cal_raw_t, &cal_t_c, &cal_raw_p, &cal_p_pa) == 0) {
          initial_pressure_pa_sum += cal_p_pa;
          initial_temperature_c_sum += cal_t_c;
          valid_calibration_readings++;
          sprintf(uart_buffer, "Calib reading %d: P=%.2f Pa, T=%.2f C\r\n", i + 1, cal_p_pa, cal_t_c);
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      } else {
          sprintf(uart_buffer, "Calibration reading %d failed.\r\n", i + 1);
          HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      }
      HAL_Delay(100); // Delay between readings (adjust based on ODR if necessary)
  }

  if (valid_calibration_readings > 0) {
      float avg_initial_pressure_pa = initial_pressure_pa_sum / valid_calibration_readings;
      float avg_initial_temperature_c = initial_temperature_c_sum / valid_calibration_readings;
      float known_initial_altitude_m = 0.0f; // Assume starting at 0m altitude for calibration

      calibrate_sea_level_pressure_hpa(avg_initial_pressure_pa / 100.0f, known_initial_altitude_m); // Convert Pa to hPa

      sprintf(uart_buffer, "BMP390 Calibrated. Avg P: %.2f Pa, Avg T: %.2f C. Sea Level P: %.2f hPa\r\n",
              avg_initial_pressure_pa, avg_initial_temperature_c, sea_level_pressure_hpa);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  } else {
      sprintf(uart_buffer, "BMP390 Calibration failed. Using default sea level pressure (1013.25 hPa).\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      sea_level_pressure_hpa = 1013.25f; // Default
  }
  
  // Initialize ADXL375 high-g accelerometer
  sprintf(uart_buffer, "Initializing ADXL375 high-g accelerometer...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  
  adxl375_init();
  
  uint8_t adxl_id = adxl375_read(ADXL375_REG_DEVID);
  if (adxl_id == 0xE5) {
      sprintf(uart_buffer, "ADXL375 initialized successfully. DEVID: 0x%02X (expected 0xE5)\r\n", adxl_id);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
      // Calibrate ADXL375 - assume device is positioned so Z reads 1G, X/Y read 0G
      sprintf(uart_buffer, "Calibrating ADXL375... Please keep device still with Z-axis up.\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
      adxl375_calibrate(); // Call the new calibration function
      
      // Read back and verify the offsets (original code)
      int8_t read_offset_x, read_offset_y, read_offset_z;
      adxl375_read_offsets(&read_offset_x, &read_offset_y, &read_offset_z);
      
      sprintf(uart_buffer, "ADXL375 Calibration Complete.\r\n");
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
      sprintf(uart_buffer, "  Readback Offsets from ADXL375: X=%d, Y=%d, Z=%d\r\n", read_offset_x, read_offset_y, read_offset_z);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
  } else {
      sprintf(uart_buffer, "ADXL375 initialization FAILED! DEVID: 0x%02X (expected 0xE5)\r\n", adxl_id);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  }
  
  // Turn on LED to indicate ready state
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET); // LD2 is usually green or yellow.
  
  // Initialize Kalman Filter
  // Noise parameters:
  // BMP390 altitude noise RMS = 0.15m => variance = 0.15*0.15 = 0.0225 m^2
  // LSM6DSO accel noise density = 160e-6 g/sqrt(Hz). For 100Hz sample rate (50Hz bandwidth):
  // accel_process_noise_variance = (160e-6 * 9.80665)^2 * 50 approx 0.000123 (m/s^2)^2
  float initial_altitude_m = 0.0f; // Will be updated by first good baro reading if possible, or use current persistent
  if (sea_level_pressure_hpa > 0.0f && persistent_bmp_pres_pa > 0.0f) { // Check if calibration ran and we have a pressure
      initial_altitude_m = calculate_altitude_hpa(persistent_bmp_pres_pa / 100.0f);
  } else if (valid_calibration_readings > 0) { // Fallback to avg calibration altitude if available
      initial_altitude_m = calculate_altitude_hpa((initial_pressure_pa_sum / valid_calibration_readings) / 100.0f);
  }

  kalman_init(&kf_altitude_velocity,
              initial_altitude_m,       // initial_altitude_m
              0.0225f,                  // initial_altitude_variance (R_baro)
              1.0f,                     // initial_velocity_variance (e.g., 1 (m/s)^2)
              0.000123f,                // process_noise_accel_variance (sigma_a^2 for Q)
              0.0225f);                 // measurement_noise_baro_variance (R_baro)

  sprintf(uart_buffer, "Kalman Filter Initialized. Initial Alt: %.2fm\r\n", initial_altitude_m);
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  // Initialize Flight State Machine
  flight_state_machine.flight_state = READY;
  flight_state_machine.state_changed = 0; // false
  flight_state_machine.apogee_flag = false;
  flight_state_machine.main_deployment_flag = false;
  flight_state_machine.thrust_trigger_time = 0; // Initialize new members
  flight_state_machine.iteration_count = 0;   // Initialize new members
  flight_state_machine.apogee_trigger_time_ms = 0; // Initialize apogee time
  for (int i = 0; i < 5; i++) {
      flight_state_machine.memory[i] = 0.0f;
  }

  // Initialize Control Settings
  control_settings.liftoff_acc_threshold = 4.0f; // 4G threshold

   // Define the flight state machine instance // This comment seems to be a leftover, FSM is already defined
  /* USER CODE END 2 */

  // Wait for arming signal (PA5 to go HIGH when disconnected from GND)
  // Loop indefinitely until PA5 is HIGH (disconnected from GND)
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) {
      HAL_Delay(1); // Optional: uncomment for a slight delay
  }

  sprintf(uart_buffer, "System ARMED (PA5 HIGH). Entering main loop.\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t loop_start_tick = HAL_GetTick(); // Record start time of the loop

    uint32_t raw_pressure; // raw_temperature is still needed for the function call
    uint32_t local_raw_temperature; // Use a distinct local variable for the raw temperature param

    // Sensor values
    float lsm_acc_x = 0.0f, lsm_acc_y = 0.0f, lsm_acc_z = 0.0f;
    float lsm_gyr_x = 0.0f, lsm_gyr_y = 0.0f, lsm_gyr_z = 0.0f;
    float bmp_temp_c = 0.0f, bmp_pres_pa = 0.0f, bmp_alt_m = 0.0f;
    float adxl_hi_g_x = 0.0f, adxl_hi_g_y = 0.0f, adxl_hi_g_z = 0.0f;

    // Madgwick inputs
    float acc_g[3];  // Acceleration in g
    float gyro_rps[3]; // Gyroscope in rad/s

    bool lsm_accel_data_ready = false;
    bool lsm_gyro_data_ready = false;
    bool bmp_data_ok = false; // Initialize to false for this cycle

    // Read LSM6DSO data
    uint8_t reg_lsm;
    lsm6dso_status_reg_get(&dev_ctx, &reg_lsm);

    if (reg_lsm & 0x01) { // Check XLDA bit
      lsm_accel_data_ready = true;
      lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      lsm_acc_x = lsm6dso_from_fs16g_to_mg(data_raw_acceleration[0]) - lsm6dso_accel_offset_mg[0];
      lsm_acc_y = lsm6dso_from_fs16g_to_mg(data_raw_acceleration[1]) - lsm6dso_accel_offset_mg[1];
      lsm_acc_z = lsm6dso_from_fs16g_to_mg(data_raw_acceleration[2]) - lsm6dso_accel_offset_mg[2];

      // Convert accelerometer data from mg to g for Madgwick
      acc_g[0] = lsm_acc_x / 1000.0f;
      acc_g[1] = lsm_acc_y / 1000.0f;
      acc_g[2] = lsm_acc_z / 1000.0f;
    }

        if (reg_lsm & 0x02) { // Check GDA (Gyroscope Data Available) bit
      lsm_gyro_data_ready = true;
            lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      lsm_gyr_x = lsm6dso_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) - lsm6dso_gyro_offset_mdps[0];
      lsm_gyr_y = lsm6dso_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) - lsm6dso_gyro_offset_mdps[1];
      lsm_gyr_z = lsm6dso_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) - lsm6dso_gyro_offset_mdps[2];

      // Convert gyroscope data from mdps to rad/s for Madgwick
      // PI is defined in Madgwick_filter.h
      gyro_rps[0] = lsm_gyr_x * (PI / 180.0f / 1000.0f);
      gyro_rps[1] = lsm_gyr_y * (PI / 180.0f / 1000.0f);
      gyro_rps[2] = lsm_gyr_z * (PI / 180.0f / 1000.0f);
    }

    // Update Madgwick filter (call this every 10ms)
    // Ensure acc_g and gyro_rps are updated if data is ready, otherwise, decide on behavior
    // (e.g., use last known, or skip update if critical data missing)
    // For now, assume data is usually ready. If not, filter might get stale inputs from previous loop.
    MadgwickAHRSupdateIMU(gyro_rps[0], gyro_rps[1], gyro_rps[2], acc_g[0], acc_g[1], acc_g[2]);

    // Calculate world-frame vertical linear acceleration for Kalman filter
    float acc_z_world_g = (2.0f*(q1*q3 - q0*q2)*acc_g[0] + 2.0f*(q2*q3 + q0*q1)*acc_g[1] + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*acc_g[2]);
    float linear_accel_z_mps2 = (acc_z_world_g - 1.0f) * 9.80665f; // Assuming Z is UP

    // Kalman Predict Step (every loop = every 10ms)
    kalman_predict(&kf_altitude_velocity, linear_accel_z_mps2, 0.01f);

    // Read BMP390 data only every other cycle (20ms interval if loop is 10ms)
    bool new_baro_data_for_kalman = false;
    if (bmp390_read_scheduler == 0) {
        float func_temp_c, func_pres_pa; // Local variables for the values read from the function
        if (bmp390_read_temperature_pressure(&bmp390_handle, &local_raw_temperature, &func_temp_c, &raw_pressure, &func_pres_pa) == 0) {
            bmp_data_ok = true; // Indicates data read attempt was successful for this cycle
            persistent_bmp_pres_pa = func_pres_pa; // Update persistent pressure
            float current_pressure_hpa = persistent_bmp_pres_pa / 100.0f;
            persistent_bmp_alt_m = calculate_altitude_hpa(current_pressure_hpa); // Update persistent altitude
            new_baro_data_for_kalman = true; // New data is available for Kalman

            // Temperature reading is no longer updated or used here actively
        HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // Toggle LD1 (usually green) to show activity
    } else {
            // bmp_data_ok remains false, set by initialization
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Turn on LD3 (usually red) for error
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET); // Blink error LED
        }
    }
    // On other cycles, bmp_data_ok remains false, and old bmp_pres_pa, bmp_alt_m are used if printed.

    // Kalman Update Step (conditionally, when new barometer data is available)
    if (new_baro_data_for_kalman) {
        kalman_update_barometer(&kf_altitude_velocity, persistent_bmp_alt_m);
    }
    
    bmp390_read_scheduler++;
    if (bmp390_read_scheduler >= 1) { // Reset every 2 cycles
        bmp390_read_scheduler = 0;
    }
    
    // Read ADXL375 high-g accelerometer (±200g range)
    int16_t adxl_raw_x, adxl_raw_y, adxl_raw_z;
    adxl375_read_xyz(&adxl_raw_x, &adxl_raw_y, &adxl_raw_z);
    adxl_hi_g_x = (float)adxl_raw_x * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f); 
    adxl_hi_g_y = (float)adxl_raw_y * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f);
    adxl_hi_g_z = (float)adxl_raw_z * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f);
    
    // Prepare data for Flight State Machine
    vf32_t adxl_acc_data_g;
    adxl_acc_data_g.x = adxl_hi_g_x;
    adxl_acc_data_g.y = adxl_hi_g_y;
    adxl_acc_data_g.z = adxl_hi_g_z;

    vf32_t lsm_gyro_data_for_fsm;
    lsm_gyro_data_for_fsm.x = gyro_rps[0];
    lsm_gyro_data_for_fsm.y = gyro_rps[1];
    lsm_gyro_data_for_fsm.z = gyro_rps[2];

    estimation_output_t current_state_estimation;
    current_state_estimation.height = kf_altitude_velocity.altitude_m;
    current_state_estimation.velocity = kf_altitude_velocity.vertical_velocity_mps;
    current_state_estimation.acceleration = linear_accel_z_mps2; // World frame Z acceleration

    // Read Launch Detect Pin (PA6)
    bool launch_pin_is_high = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);

    // Check and update flight phase
    check_flight_phase(&flight_state_machine, adxl_acc_data_g, lsm_gyro_data_for_fsm, current_state_estimation, &control_settings, launch_pin_is_high);

    uint32_t loop_end_tick = HAL_GetTick();
    uint32_t execution_time_ms = loop_end_tick - loop_start_tick;

    // Check if execution time exceeds 10ms
    if (execution_time_ms >= TARGET_LOOP_PERIOD_MS) {
        // Blink LED3 if the execution time exceeds 10ms
        if (HAL_GetTick() - led3_last_toggle_time >= led3_blink_interval) {
            HAL_GPIO_TogglePin(GPIOB, LD3_Pin); // Toggle LED3
            led3_last_toggle_time = HAL_GetTick(); // Update last toggle time
        }
    }

    // Consolidate UART output
    int_fast16_t current_len = 0;
    bool data_printed_prev = false; // Flag to track if a separator is needed

#ifdef PRINT_LSM6DSO_ACCEL_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "A:");
    if (lsm_accel_data_ready) {
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "%.1f,%.1f,%.1f", lsm_acc_x, lsm_acc_y, lsm_acc_z);
    } else {
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "NR,NR,NR");
    }
    data_printed_prev = true;
#endif

#ifdef PRINT_LSM6DSO_GYRO_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "G:");
    if (lsm_gyro_data_ready) {
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "%.1f,%.1f,%.1f", lsm_gyr_x, lsm_gyr_y, lsm_gyr_z);
    } else {
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "NR,NR,NR");
    }
    data_printed_prev = true;
#endif

#ifdef PRINT_BMP390_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "B:");
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "P%.0f,Alt%.1f", persistent_bmp_pres_pa, persistent_bmp_alt_m);
    data_printed_prev = true;
#endif

#ifdef PRINT_ADXL375_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "H:%.1f,%.1f,%.1f", adxl_hi_g_x, adxl_hi_g_y, adxl_hi_g_z);
    data_printed_prev = true;
#endif
    
#ifdef PRINT_QUATERNION_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "Q:%.2f,%.2f,%.2f,%.2f", q0, q1, q2, q3);
    data_printed_prev = true;
#endif

#ifdef PRINT_LOOP_EXEC_TIME
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "L:%lums%s",
                            execution_time_ms,
                            (execution_time_ms >= TARGET_LOOP_PERIOD_MS && execution_time_ms != 0) ? " OV!" : "");
    data_printed_prev = true; // Though this is last, set it for consistency
#endif

#ifdef PRINT_KALMAN_DATA
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 0) { current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|"); }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "KF:Alt%.2f,Vel%.2f", kf_altitude_velocity.altitude_m, kf_altitude_velocity.vertical_velocity_mps);
    data_printed_prev = true;
#endif

    // Flight Phase Print
    if (data_printed_prev && current_len > 0 && (sizeof(uart_buffer) - current_len) > 2) { // Need space for |FP:X
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "|");
    }
    char phase_char = 'U'; // Unknown/default
    switch (flight_state_machine.flight_state) {
        case READY:     phase_char = 'R'; break;
        case THRUSTING: phase_char = 'T'; break;
        case COASTING:  phase_char = 'C'; break;
        case DROGUE:    phase_char = 'D'; break;
        case MAIN:      phase_char = 'M'; break;
        case TOUCHDOWN: phase_char = 'L'; break; // L for Landed/Touchdown
    }
    current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "FP:%c", phase_char);
    // No need to set data_printed_prev = true; as this is the last item before newline typically

    // Add newline if any data was printed
    if (current_len > 0) {
        current_len += snprintf(uart_buffer + current_len, sizeof(uart_buffer) - current_len, "\r\n");
    }

    if (current_len > 0 && (size_t)current_len < sizeof(uart_buffer)) { // Check if anything was written and buffer not overflowed by snprintf
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, current_len, HAL_MAX_DELAY);
    } else if ((size_t)current_len >= sizeof(uart_buffer)) {
        // Handle potential truncation / error in string formatting if buffer was too small
        char err_msg[] = "UART buffer overflow!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
    }


    if (execution_time_ms < TARGET_LOOP_PERIOD_MS) {
      uint32_t delay_ms = TARGET_LOOP_PERIOD_MS - execution_time_ms;
      HAL_Delay(delay_ms);
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // __HAL_RCC_GPIOF_CLK_ENABLE(); /* Clock for PF12, can be removed if PF12 is no longer used */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 - REMOVE/COMMENT OUT if no longer used */
  // GPIO_InitStruct.Pin = GPIO_PIN_12;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 for Arming Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 for Launch Detect */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
