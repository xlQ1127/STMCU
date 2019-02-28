#include "BSP_BMP280.h"

void bmp280_init(void)
{

    if (I2C_CheckDevice(BMP280_ADDRESS) == 1)
        ;

    {
        bmp280_id = bmp280_read_register(BMP280_CHIPID_REG);
    }
    if (bmp280_id == 0x58)
    {
        bmp280_write_register(BMP280_RESET_REG, BMP280_RESET_VALUE); //reset
        reset_read = bmp280_read_register(BMP280_RESET_REG);

        bmp280_write_register(BMP280_CTRLMEAS_REG, 0xff); //ctrl
        bmp280_write_register(BMP280_CONFIG_REG, 0x00);   //config

        config_reg = bmp280_read_register(BMP280_CONFIG_REG);
        ctrlmeas_reg = bmp280_read_register(BMP280_CTRLMEAS_REG);
    }

    else
    {
        while (1)
        {
        };
    }

    lsb = bmp280_read_register(BMP280_DIG_T1_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_T1_MSB_REG);
    T1 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_T2_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_T2_MSB_REG);
    T2 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_T3_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_T3_MSB_REG);
    T3 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P1_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P1_MSB_REG);
    P1 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P2_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P2_MSB_REG);
    P2 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P3_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P3_MSB_REG);
    P3 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P4_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P4_MSB_REG);
    P4 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P5_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P5_MSB_REG);
    P5 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P6_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P6_MSB_REG);
    P6 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P7_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P7_MSB_REG);
    P7 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P8_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P8_MSB_REG);
    P8 = (msb << 8) | lsb;

    lsb = bmp280_read_register(BMP280_DIG_P9_LSB_REG);
    msb = bmp280_read_register(BMP280_DIG_P9_MSB_REG);
    P9 = (msb << 8) | lsb;
}

double bmp280_get_temperature(void)
{

    t_xlsb = bmp280_read_register(BMP280_TEMPERATURE_XLSB_REG);
    t_lsb = bmp280_read_register(BMP280_TEMPERATURE_LSB_REG);
    t_msb = bmp280_read_register(BMP280_TEMPERATURE_MSB_REG);
    adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);

    var1_t = (((double)adc_T) / 16384.0 - ((double)T1) / 1024.0) * ((double)T2);
    var2_t = ((((double)adc_T) / 131072.0 - ((double)T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)T1) / 8192.0)) * ((double)T3);

    t_fine = (int32_t)(var1_t + var2_t);

    temperature = (var1_t + var2_t) / 5120.0;

    return temperature;
}

/* Returns pressure in Pa as double. Output value of ?6386.2?equals 96386.2 Pa = 963.862 hPa */
double bmp280_get_pressure(void)
{

    p_lsb = bmp280_read_register(BMP280_PRESSURE_LSB_REG);
    p_msb = bmp280_read_register(BMP280_PRESSURE_MSB_REG);
    p_xlsb = bmp280_read_register(BMP280_PRESSURE_XLSB_REG);
    adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);

    var1_p = ((double)t_fine / 2.0) - 64000.0;
    var2_p = var1_p * var1_p * ((double)P6) / 32768.0;
    var2_p = var2_p + var1_p * ((double)P5) * 2.0;
    var2_p = (var2_p / 4.0) + (((double)P4) * 65536.0);
    var1_p = (((double)P3) * var1_p * var1_p / 524288.0 + ((double)P2) * var1_p) / 524288.0;
    var1_p = (1.0 + var1_p / 32768.0) * ((double)P1);
    if (var1_p == 0.0)
    {
        return 0; // avoid exception caused by division by zero
    }
    pressure = 1048576.0 - (double)adc_P;
    pressure = (pressure - (var2_p / 4096.0)) * 6250.0 / var1_p;
    var1_p = ((double)P9) * pressure * pressure / 2147483648.0;
    var2_p = pressure * ((double)P8) / 32768.0;
    pressure = pressure + (var1_p + var2_p + ((double)P7)) / 16.0;

    return pressure;
}