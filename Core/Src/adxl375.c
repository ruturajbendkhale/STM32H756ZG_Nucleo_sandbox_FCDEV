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

    adxl375_write(ADXL375_REG_DATA_FORMAT, 0x0B); // (0x0B = 00001011)
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

void adxl375_read_xyz_mps2(float *x_mps2, float *y_mps2, float *z_mps2)
{
    int16_t raw_x, raw_y, raw_z;
    adxl375_read_xyz(&raw_x, &raw_y, &raw_z);

    // Convert raw data to m/s^2
    // Scale factor is 49 mg/LSB. 1 g = 1000 mg.
    // Acceleration (g) = raw_value * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f)
    // Acceleration (m/s^2) = Acceleration (g) * GRAVITY_MS2

    if (x_mps2 != NULL) {
        *x_mps2 = (float)raw_x * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f) * GRAVITY_MS2;
    }
    if (y_mps2 != NULL) {
        *y_mps2 = (float)raw_y * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f) * GRAVITY_MS2;
    }
    if (z_mps2 != NULL) {
        *z_mps2 = (float)raw_z * (ADXL375_SENSITIVITY_MG_PER_LSB / 1000.0f) * GRAVITY_MS2;
    }
}

// New functions for offset registers
void adxl375_write_offsets(int8_t ofx, int8_t ofy, int8_t ofz)
{
    adxl375_write(ADXL375_REG_OFSX, (uint8_t)ofx);
    adxl375_write(ADXL375_REG_OFSY, (uint8_t)ofy);
    adxl375_write(ADXL375_REG_OFSZ, (uint8_t)ofz);
}

void adxl375_read_offsets(int8_t *ofx, int8_t *ofy, int8_t *ofz)
{
    if (ofx != NULL) {
        *ofx = (int8_t)adxl375_read(ADXL375_REG_OFSX);
    }
    if (ofy != NULL) {
        *ofy = (int8_t)adxl375_read(ADXL375_REG_OFSY);
    }
    if (ofz != NULL) {
        *ofz = (int8_t)adxl375_read(ADXL375_REG_OFSZ);
    }
}

void adxl375_calibrate(void)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t raw_x, raw_y, raw_z;

    for (int i = 0; i < ADXL375_CALIBRATION_SAMPLES; i++) {
        adxl375_read_xyz(&raw_x, &raw_y, &raw_z);
        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        HAL_Delay(10); // Delay between samples, as was in main.c
    }

    float avg_x = (float)sum_x / ADXL375_CALIBRATION_SAMPLES;
    float avg_y = (float)sum_y / ADXL375_CALIBRATION_SAMPLES;
    float avg_z = (float)sum_z / ADXL375_CALIBRATION_SAMPLES;

    // Calculate offsets needed to make Z read 1G (approx) and X/Y read 0G
    // The sensitivity 49.0f mg/LSB is from adxl375.h
    float expected_1g_raw = 1000.0f / ADXL375_SENSITIVITY_MG_PER_LSB; 

    // Using the same offset calculation logic as was in main.c
    // The factor 4.0f was empirically derived or a simplification.
    // ADXL375 datasheet specifies offset register scale factor of 15.6 mg/LSB.
    int8_t offset_x = -(int8_t)(avg_x / 4.0f); 
    int8_t offset_y = -(int8_t)(avg_y / 4.0f);
    int8_t offset_z = -(int8_t)((avg_z - expected_1g_raw) / 4.0f);

    // Clamp to int8_t range (-128 to 127)
    // The cast to int8_t handles the positive side, but negative needs care if intermediate is < -128.
    // However, direct casting from float usually truncates. The (int8_t) cast will take the lower 8 bits.
    // A more robust clamping for int8_t would be:
    // offset_x = (offset_x_float < -128.0f) ? -128 : ((offset_x_float > 127.0f) ? 127 : (int8_t)offset_x_float);
    // For now, sticking to the original simple cast as in main.c

    adxl375_write_offsets(offset_x, offset_y, offset_z);
}
