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
// #include "lsm6dso.h" // Commented out as LSM6DSO sensor is removed
#include "driver_bmp390.h" // New BMP390 driver header
#include "adxl375.h"      // ADXL375 accelerometer header
#include <math.h>          // For powf in altitude calculation
#include <stdarg.h>        // For vsnprintf in debug print

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
// static stmdev_ctx_t dev_ctx; // Commented out LSM6DSO context

// bmp390_handle_t bmp390_handle; // BMP390 related - COMMENTED OUT
char uart_buffer[256]; // Increased buffer size
// float sea_level_pressure_hpa; // Store as hPa for altitude calculations - BMP390 related - COMMENTED OUT
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

// Wrapper functions for BMP390 driver (Commented out)
/*
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

// Dummy SPI functions to satisfy driver checks when using I2C (Commented out)
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
    // Prepend "BMP390_DBG: " to distinguish driver debug messages
    // snprintf(uart_buffer, sizeof(uart_buffer), "BMP390_DBG: %s", dbg_buffer); // This might truncate
    // HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    // Direct transmit is simpler if buffer is managed carefully
    HAL_UART_Transmit(&huart3, (uint8_t*)"BMP390_DBG: ", 12, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buffer, strlen(dbg_buffer), HAL_MAX_DELAY);
}


// Altitude calculation functions (Commented out)
void calibrate_sea_level_pressure_hpa(float current_pressure_hpa, float known_altitude_meters) {
  sea_level_pressure_hpa = current_pressure_hpa / powf((1.0f - (known_altitude_meters * 0.0000225577f)), 5.255877f);
}

// pressure_hpa: current measured pressure in hPa
// returns altitude in meters
float calculate_altitude_hpa(float pressure_hpa) {
  if (sea_level_pressure_hpa <= 0) return 0.0f; // Avoid division by zero or log of non-positive
  return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_pressure_hpa, 0.1903f));
}
*/

void calibrate_adxl375_offsets(void);

// Helper function to clamp a float to int8_t range
static int8_t clamp_int8(float val) {
    if (val > 127.0f) return 127;
    if (val < -128.0f) return -128;
    return (int8_t)val;
}

void calibrate_adxl375_offsets(void) {
    const int num_readings = 50;
    float sum_x_mps2 = 0.0f, sum_y_mps2 = 0.0f, sum_z_mps2 = 0.0f;
    float avg_x_mps2, avg_y_mps2, avg_z_mps2;
    float offset_x_mps2, offset_y_mps2, offset_z_mps2;
    float offset_x_mg, offset_y_mg, offset_z_mg;
    float offset_x_lsb, offset_y_lsb, offset_z_lsb;
    int8_t reg_ofx, reg_ofy, reg_ofz;

    sprintf(uart_buffer, "Starting ADXL375 offset calibration (ensure sensor is stable, Z-axis up)...\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // 1. Reset hardware offsets to 0 to measure current raw offsets
    adxl375_write_offsets(0, 0, 0);
    HAL_Delay(20); // Give it a moment

    sprintf(uart_buffer, "Taking %d readings for averaging...\r\n", num_readings);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // 2. Read and average multiple readings
    for (int i = 0; i < num_readings; i++) {
        float cur_x, cur_y, cur_z;
        adxl375_read_xyz_mps2(&cur_x, &cur_y, &cur_z);
        sum_x_mps2 += cur_x;
        sum_y_mps2 += cur_y;
        sum_z_mps2 += cur_z;
        HAL_Delay(20); // Adjusted delay, ADXL375 ODR is 100Hz (10ms period), 20ms is safe
    }
    avg_x_mps2 = sum_x_mps2 / num_readings;
    avg_y_mps2 = sum_y_mps2 / num_readings;
    avg_z_mps2 = sum_z_mps2 / num_readings;

    sprintf(uart_buffer, "Avg raw readings (before cal): X=%.2f, Y=%.2f, Z=%.2f m/s^2\r\n", avg_x_mps2, avg_y_mps2, avg_z_mps2);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // 3. Calculate desired offsets (assuming Z is up, X & Y are horizontal)
    // For X and Y, the current average IS the offset we want to remove.
    offset_x_mps2 = avg_x_mps2;
    offset_y_mps2 = avg_y_mps2;
    // For Z, we want it to read GRAVITY_MS2 (approx 9.81). So, offset = current_avg - target_value
    offset_z_mps2 = avg_z_mps2 - GRAVITY_MS2; // If Z points up, its reading should be +GRAVITY_MS2

    sprintf(uart_buffer, "Calculated m/s^2 offsets to remove: X=%.2f, Y=%.2f, Z(comp for G)=%.2f\r\n", offset_x_mps2, offset_y_mps2, offset_z_mps2);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // 4. Convert offsets from m/s^2 to mg, then to LSB for OFS registers
    const float adxl375_offset_lsb_mg_datasheet = 15.6f;
    // Empirically observed amplification factor of the offset register's effect
    const float empirical_amplification_factor = 12.5f; // Derived from (12.01+12.73+12.78)/3
    const float actual_offset_mg_per_lsb = adxl375_offset_lsb_mg_datasheet * empirical_amplification_factor;

    offset_x_mg = (offset_x_mps2 / GRAVITY_MS2) * 1000.0f;
    offset_y_mg = (offset_y_mps2 / GRAVITY_MS2) * 1000.0f;
    offset_z_mg = (offset_z_mps2 / GRAVITY_MS2) * 1000.0f;

    // Use the effective actual_offset_mg_per_lsb for conversion
    offset_x_lsb = offset_x_mg / actual_offset_mg_per_lsb;
    offset_y_lsb = offset_y_mg / actual_offset_mg_per_lsb;
    offset_z_lsb = offset_z_mg / actual_offset_mg_per_lsb;

    // 5. Write the NEGATIVE of the LSB values to the offset registers, clamping to int8_t range
    reg_ofx = clamp_int8(-offset_x_lsb);
    reg_ofy = clamp_int8(-offset_y_lsb);
    reg_ofz = clamp_int8(-offset_z_lsb);

    sprintf(uart_buffer, "Values to write to OFS registers (LSB, 15.6mg/LSB): OFSX=%d, OFSY=%d, OFSZ=%d\r\n", reg_ofx, reg_ofy, reg_ofz);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    adxl375_write_offsets(reg_ofx, reg_ofy, reg_ofz);
    HAL_Delay(10); // Short delay after writing offsets

    sprintf(uart_buffer, "ADXL375 offset calibration complete. New offsets written to sensor.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // Optional: Read back offsets written to sensor to verify
    int8_t rofx, rofy, rofz;
    adxl375_read_offsets(&rofx, &rofy, &rofz);
    sprintf(uart_buffer, "Read back from OFS registers: OFSX=%d, OFSY=%d, OFSZ=%d\r\n", rofx, rofy, rofz);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // Optional: Take a few readings after calibration to see the effect
    sprintf(uart_buffer, "Taking a few readings post-calibration...\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    for(int k=0; k<3; k++){
        float post_cal_x, post_cal_y, post_cal_z;
        adxl375_read_xyz_mps2(&post_cal_x, &post_cal_y, &post_cal_z);
        sprintf(uart_buffer, "Post-Cal Reading %d: X=%.2f, Y=%.2f, Z=%.2f m/s^2\r\n", k+1, post_cal_x, post_cal_y, post_cal_z);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        HAL_Delay(100);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define LSM6DSO_I2C_ADD_L for future use when LSM6DSO is re-integrated
#define LSM6DSO_I2C_ADD_L 0x6A  // Standard I2C address for LSM6DSO (0x6A when SDO/SA0 is connected to GND)

// LSM6DSO functions kept but commented out for future use
/*
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write((I2C_HandleTypeDef*)handle, LSM6DSO_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read((I2C_HandleTypeDef*)handle, LSM6DSO_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}
*/

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
    if (i < 0x08 || i > 0x77) continue; // Valid 7-bit addresses

    HAL_StatusTypeDef i2c_result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 10); // Use 2 trials, 10ms timeout
    
    if (i2c_result == HAL_OK) {
      i2c_devices_found++;
      sprintf(uart_buffer, "Found I2C device at address: 0x%02X\r\n", i);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      
      /* // BMP390 specific check commented out
      if (i == (BMP390_I2C_ADDRESS_LOW >> 1) || i == (BMP390_I2C_ADDRESS_HIGH >> 1)) {
        sprintf(uart_buffer, "  --> This could be a BMP390 sensor!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      }
      */
      if (i == (ADXL375_ADDRESS >> 1)) {
        sprintf(uart_buffer, "  --> This could be an ADXL375 sensor!\r\n");
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

  // Initialize ADXL375
  sprintf(uart_buffer, "Initializing ADXL375...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  adxl375_init();
  uint8_t adxl_chip_id = adxl375_read(ADXL375_REG_DEVID);
  sprintf(uart_buffer, "ADXL375 Chip ID: 0x%02X (Expected: 0xE5)\r\n", adxl_chip_id);
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  if (adxl_chip_id == 0xE5) {
    sprintf(uart_buffer, "ADXL375 Initialized Successfully!\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // Calibrate ADXL375 offsets
    calibrate_adxl375_offsets();

  } else {
    sprintf(uart_buffer, "ADXL375 Initialization FAILED! Chip ID: 0x%02X. Check wiring or device.\r\n", adxl_chip_id);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    // Optionally, call Error_Handler() or handle as appropriate
  }

  /* // BMP390 Initialization and Configuration Code (Commented out)
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
      if (bmp390_set_odr(&bmp390_handle, BMP390_ODR_25_HZ) != 0) { // 25 Hz ODR
          sprintf(uart_buffer, "BMP390: Failed to set ODR\r\n");
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
  */
  
  // Turn on LED to indicate ready state
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET); // LD2 is usually green or yellow.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* // BMP390 Data Reading Code (Commented out)
    uint32_t raw_pressure, raw_temperature;
    float pressure_pa, temperature_c;
    float altitude_m;

    if (bmp390_read_temperature_pressure(&bmp390_handle, &raw_temperature, &temperature_c, &raw_pressure, &pressure_pa) == 0) {
        float current_pressure_hpa = pressure_pa / 100.0f;
        altitude_m = calculate_altitude_hpa(current_pressure_hpa);

        sprintf(uart_buffer, "T: %.2f C, P: %.2f Pa, Alt: %.2f m\r\n",
                temperature_c, pressure_pa, altitude_m);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        
        HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // Toggle LD1 (usually green) to show activity
    } else {
        sprintf(uart_buffer, "Error reading BMP390 (new driver)\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET); // Turn on LD3 (usually red) for error
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
    */

    // ADXL375 Data Reading
    float ax_mps2, ay_mps2, az_mps2;
    adxl375_read_xyz_mps2(&ax_mps2, &ay_mps2, &az_mps2);

    sprintf(uart_buffer, "%0.2f, %0.2f, %0.2f\r\n",
            ax_mps2, ay_mps2, az_mps2);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // Toggle LD1 (usually green) to show activity
    
    /* LSM6DSO code commented out as sensor is removed */
    
    HAL_Delay(200); // Delay for readability, adjust as needed. ADXL375 ODR is 100Hz by default.
    
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

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
