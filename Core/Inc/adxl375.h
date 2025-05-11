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

// Measurement range for ADXL375 is ±200g
// The device outputs 11-bit data with scale factor 49 mg/LSB

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

#endif /* __ADXL375_H */
