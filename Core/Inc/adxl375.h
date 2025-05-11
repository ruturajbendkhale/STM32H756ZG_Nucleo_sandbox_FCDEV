#ifndef __ADXL375_H
#define __ADXL375_H

#include "main.h"

// ADXL375 I2C address (ALT ADDRESS pin low = 0x53 << 1)
#define ADXL375_ADDRESS        (0x53 << 1)

// ADXL375 Register Addresses
#define ADXL375_REG_DEVID      0x00
#define ADXL375_REG_POWER_CTL  0x2D
#define ADXL375_REG_DATAX0     0x32
#define ADXL375_REG_DATAX1     0x33
#define ADXL375_REG_DATAY0     0x34
#define ADXL375_REG_DATAY1     0x35
#define ADXL375_REG_DATAZ0     0x36
#define ADXL375_REG_DATAZ1     0x37
#define ADXL375_REG_DATA_FORMAT 0x31
#define ADXL375_REG_BW_RATE    0x2C
// New Offset Register Definitions
#define ADXL375_REG_OFSX       0x1E
#define ADXL375_REG_OFSY       0x1F
#define ADXL375_REG_OFSZ       0x20

// Measurement range for ADXL375 is ±200g
// The device outputs data with a typical scale factor of 49 mg/LSB.
#define ADXL375_SENSITIVITY_MG_PER_LSB 49.0f // mg per LSB
#define GRAVITY_MS2                    9.80665f // Standard gravity in m/s^2

// I2C handle (should be defined in main.c or elsewhere)
extern I2C_HandleTypeDef hi2c1;

// Function declarations
void     adxl375_write(uint8_t reg, uint8_t value);
uint8_t  adxl375_read(uint8_t reg);
void     adxl375_read_xyz(int16_t *x, int16_t *y, int16_t *z);
void     adxl375_init(void);
int16_t  adxl375_read_x(void);
int16_t  adxl375_read_y(void);
int16_t  adxl375_read_z(void);
void     adxl375_read_xyz_mps2(float *x_mps2, float *y_mps2, float *z_mps2);
// New functions for offset registers
void     adxl375_write_offsets(int8_t ofx, int8_t ofy, int8_t ofz);
void     adxl375_read_offsets(int8_t *ofx, int8_t *ofy, int8_t *ofz);

#endif /* __ADXL375_H */
