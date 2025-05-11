#include "adxl375.h"

uint8_t data_rec[6];
uint8_t chipid = 0;

// Optional: for serial debug display
char x_char[6], y_char[6], z_char[6];

void adxl375_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(&hi2c1, ADXL375_ADDRESS, data, 2, HAL_MAX_DELAY);
}

uint8_t adxl375_read(uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, ADXL375_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

void adxl375_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    HAL_I2C_Mem_Read(&hi2c1, ADXL375_ADDRESS, ADXL375_REG_DATAX0, I2C_MEMADD_SIZE_8BIT, data_rec, 6, HAL_MAX_DELAY);

    *x = (int16_t)((data_rec[1] << 8) | data_rec[0]);
    *y = (int16_t)((data_rec[3] << 8) | data_rec[2]);
    *z = (int16_t)((data_rec[5] << 8) | data_rec[4]);
}

void adxl375_init(void)
{
    chipid = adxl375_read(ADXL375_REG_DEVID);  // Read device ID (should be 0xE5)

    adxl375_write(ADXL375_REG_DATA_FORMAT, 0x0B); // Full resolution, right-justified, ±200g (0x0B = 00001011)
    adxl375_write(ADXL375_REG_BW_RATE, 0x0A);     // Set Output Data Rate to 100 Hz (0x0A = 00001010)
    adxl375_write(ADXL375_REG_POWER_CTL, 0x08);   // Set Measure bit (00001000)
}

int16_t adxl375_read_x(void)
{
    int16_t x;
    adxl375_read_xyz(&x, NULL, NULL);
    return x;
}

int16_t adxl375_read_y(void)
{
    int16_t y;
    adxl375_read_xyz(NULL, &y, NULL);
    return y;
}

int16_t adxl375_read_z(void)
{
    int16_t z;
    adxl375_read_xyz(NULL, NULL, &z);
    return z;
}
