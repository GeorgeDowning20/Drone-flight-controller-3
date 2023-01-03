#include "main.h"
#include "string.h"
#include "math.h"
#include "Baro.h"
#include "AERS.h"
#include "AHRS.h"

const float kT = 253952; // 16 times (Standard)
const float kP = 253952; // 16 times (Standard)

I2C_HandleTypeDef *baro_hi2c;

void configure_baro (I2C_HandleTypeDef *hi2c)
{
	baro_hi2c = hi2c;

DPS310_tx_buffer[0]=DPS310_RESET_BIT_SOFT_RST;
HAL_I2C_Mem_Write(baro_hi2c, DPS310_I2C_ADDR|I2C_write, DPS310_REG_RESET, 1, &DPS310_tx_buffer, 1, 1000);

HAL_Delay(100);

//baro doesnt like if coff read in one go
HAL_I2C_Mem_Read(baro_hi2c, DPS310_I2C_ADDR, DPS310_REG_COEF, 1, &coef,9, 1000);
HAL_Delay(10);

	HAL_I2C_Mem_Read(baro_hi2c, DPS310_I2C_ADDR, DPS310_REG_COEF+9, 1, &coef[9],9, 1000);
	HAL_Delay(10);

c0 = (int16_t)getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);
c1 = (int16_t)getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);
c00 = (int32_t)getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);
c10 = (int32_t)getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);
c01 = (int16_t)getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
c11 = (int16_t)getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
c20 = (int16_t)getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
c21 = (int16_t)getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
c30 = (int16_t)getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

DPS310_tx_buffer[0]=DPS310_TMP_CFG_BIT_TMP_RATE_32HZ | DPS310_TMP_CFG_BIT_TMP_PRC_16|0x80;
HAL_I2C_Mem_Write(baro_hi2c, DPS310_I2C_ADDR|I2C_write, DPS310_REG_TMP_CFG, 1, &DPS310_tx_buffer, 1, 1000);
HAL_Delay(10);

	DPS310_tx_buffer[0]=DPS310_PRS_CFG_BIT_PM_RATE_32HZ | DPS310_PRS_CFG_BIT_PM_PRC_16;
	HAL_I2C_Mem_Write(baro_hi2c, DPS310_I2C_ADDR|I2C_write, DPS310_REG_PRS_CFG, 1, &DPS310_tx_buffer, 1, 1000);
	HAL_Delay(10);

		DPS310_tx_buffer[0]=DPS310_CFG_REG_BIT_T_SHIFT | DPS310_CFG_REG_BIT_P_SHIFT;
		HAL_I2C_Mem_Write(baro_hi2c, DPS310_I2C_ADDR|I2C_write, DPS310_REG_CFG_REG, 1, &DPS310_tx_buffer, 1, 1000);
		HAL_Delay(10);

			DPS310_tx_buffer[0]=DPS310_MEAS_CFG_MEAS_CTRL_CONT;
			HAL_I2C_Mem_Write(baro_hi2c, DPS310_I2C_ADDR|I2C_write, DPS310_REG_MEAS_CFG, 1, &DPS310_tx_buffer, 1, 1000);



for (int i;i<5;i++)
{
HAL_I2C_Mem_Read(hi2c, DPS310_I2C_ADDR, DPS310_REG_PSR_B2, 1, &DPS310_rx_buffer,6, 1000);
Praw_sc = ((int32_t)getTwosComplement((DPS310_rx_buffer[0] << 16) + (DPS310_rx_buffer[1] << 8) + DPS310_rx_buffer[2], 24)) / kP;
Traw_sc = ((int32_t)getTwosComplement((DPS310_rx_buffer[3] << 16) + (DPS310_rx_buffer[4] << 8) + DPS310_rx_buffer[5], 24)) / kT;
pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);
temperature = c0 * 0.5f + c1 * Traw_sc;
HAL_Delay(30);

}

inital_pressure = pressure;
alt_est = 0;
alt = 0;
//b1 = pressure;
}



 int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    else {
        return raw;
    }
}


void read_baro (void)
{
		HAL_I2C_Mem_Read(baro_hi2c, DPS310_I2C_ADDR, DPS310_REG_PSR_B2, 1, &DPS310_rx_buffer,6, 1000);
		Praw_sc = ((int32_t)getTwosComplement((DPS310_rx_buffer[0] << 16) + (DPS310_rx_buffer[1] << 8) + DPS310_rx_buffer[2], 24)) / kP;
		Traw_sc = ((int32_t)getTwosComplement((DPS310_rx_buffer[3] << 16) + (DPS310_rx_buffer[4] << 8) + DPS310_rx_buffer[5], 24)) / kT;
	    pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);
	    temperature = c0 * 0.5f + c1 * Traw_sc;
	    NEW_BARO = 1;
}

