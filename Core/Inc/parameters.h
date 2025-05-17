#ifndef PARAMETERS_H_
#define PARAMETERS_H_
// CONSTANTS 

#define MILIDEGREE_TO_RADIANS PI / 180.0f / 1000.0f
// Target Loop & General Timing
#define TARGET_LOOP_PERIOD_MS 10         // Target main loop period in milliseconds (e.g., for 100Hz operation)
#define BMP_TEMP_READ_INTERVAL_LOOPS 100 // Read BMP390 temperature every N main loops (e.g., 100 loops * 10ms/loop = 1s interval)
#define LED3_BLINK_INTERVAL_MS 200       // Blink interval for LED3 in milliseconds

// I2C Addresses
#define BMP390_I2C_ADDRESS_LOW_SHIFTED  (0x76 << 1) // BMP390 I2C address when SDO is LOW (8-bit)
#define BMP390_I2C_ADDRESS_HIGH_SHIFTED (0x77 << 1) // BMP390 I2C address when SDO is HIGH (8-bit)
#define LSM6DSO_I2C_ADD_L    0x6A        // LSM6DSO I2C address when SA0/SDO is LOW (7-bit)
#define LSM6DSO_I2C_ADD_H   0x6B        // LSM6DSO I2C address when SA0/SDO is HIGH (7-bit)
#define ADXL375_I2C_ADDRESS_SHIFTED     (0x53 << 1) // ADXL375 I2C address (ALT ADDRESS pin low) (8-bit)

// Sensor Calibration & Configuration
#define ADXL375_SENSITIVITY_MG_PER_LSB  49.0f       // ADXL375 sensitivity in mg/LSB (typically 49mg/LSB for +-200g)
#define ADXL375_CALIBRATION_SAMPLES     50          // Number of samples for ADXL375 offset calibration
#define LSM6DSO_CALIBRATION_SAMPLES     100         // Number of samples for LSM6DSO offset calibration
#define STANDARD_GRAVITY_MPS2           9.80665f    // Standard acceleration due to gravity in m/s^2
#define LSM6DSO_16G_SENSITIVITY_CORR    0.488f      // LSM6DSO sensitivity for ±16g full scale (mg/LSB) with correction factor
#define LSM6DSO_SENSITIVITY_CORR_FACTOR 2.0f        // Correction factor for LSM6DSO acceleration readings
#define LSM6DSO_250DPS_SENSITIVITY      8.75f       // LSM6DSO sensitivity for ±250dps full scale (mdps/LSB)
#define LSM6DSO_32G_SENSITIVITY         0.976f      // LSM6DSO sensitivity for ±32g full scale (mg/LSB)
#define LSM6DSO_2000DPS_SENSITIVITY     70.0f       // LSM6DSO sensitivity for ±2000dps full scale (mdps/LSB)

// Flight Control Parameters
//Each Cycle execute for 10ms
#define FLIGHT_LIFTOFF_ACCEL_THRESHOLD_G        4.0f    // Acceleration magnitude (G) to detect liftoff
#define FLIGHT_LIFTOFF_SAFETY_ITERATIONS        10      // Number of consecutive loops accel must exceed threshold for liftoff 
#define FLIGHT_COASTING_SAFETY_ITERATIONS       10      // Number of consecutive loops for coasting detection criteria 
#define FLIGHT_APOGEE_SAFETY_ITERATIONS         30      // Number of consecutive loops for apogee detection criteria (velocity < 0) 
#define FLIGHT_MAIN_DEPLOY_ALTITUDE_M           500.0f  // Altitude (meters) below which main parachute can deploy (altitude condition)
#define FLIGHT_MAIN_DEPLOY_SAFETY_ITERATIONS    30      // Number of consecutive loops for main deployment altitude criteria
#define FLIGHT_MAIN_DEPLOY_DELAY_AFTER_APOGEE_MS 5000   // Delay (milliseconds) after apogee detection before main parachute can deploy (time condition)
#define FLIGHT_TOUCHDOWN_VELOCITY_THRESHOLD_MPS 3.0f    // Velocity magnitude (m/s) below which touchdown is considered
#define FLIGHT_TOUCHDOWN_SAFETY_ITERATIONS      100     // Number of consecutive loops velocity must be below threshold for touchdown

// Motor Control Parameters
#define MOTOR_NOSECONE_DURATION_MS 1000 // Duration (milliseconds) for nosecone separation motor activation
#define MOTOR_MAIN_DURATION_MS     2000 // Duration (milliseconds) for main parachute deployment motor activation
#define MOTOR_NOSECONE_PIN         GPIO_PIN_4 // PB4 for nosecone separation motor
#define MOTOR_MAIN_PIN             GPIO_PIN_5 // PB5 for main parachute deployment motor

// SD Card & Logging
#define SD_BLOCK_SIZE           512  // SD card block size in bytes
#define MAX_LOG_ENTRIES_DEBUG   100  // Maximum number of log entries for debugging/testing SD card logging

// Kalman Filter Parameters
#define KF_INITIAL_ALTITUDE_VARIANCE      0.0225f  // Initial altitude variance (R_baro) (m^2)
#define KF_INITIAL_VELOCITY_VARIANCE      1.0f     // Initial velocity variance ((m/s)^2)
#define KF_PROCESS_NOISE_ACCEL_VARIANCE   0.000123f // Process noise acceleration variance (sigma_a^2 for Q) ((m/s^2)^2)
#define KF_MEASUREMENT_NOISE_BARO_VARIANCE 0.0225f  // Measurement noise barometer variance (R_baro) (m^2)

// Barometer Constants
#define BMP_DEFAULT_SEA_LEVEL_PRESSURE_HPA 1013.25f // Default sea level pressure in hPa (hectopascals)
#define BMP_ALTITUDE_FORMULA_EXPONENT      0.1903f  // Exponent in altitude formula
#define BMP_ALTITUDE_FORMULA_FACTOR        44330.0f // Factor in altitude formula
#define BMP_ALTITUDE_FACTOR_1              0.0000225577f // Altitude factor 1 in pressure calculation
#define BMP_ALTITUDE_FACTOR_2              5.255877f // Altitude factor 2 in pressure calculation

#endif /* PARAMETERS_H_ */
