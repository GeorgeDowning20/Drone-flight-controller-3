
#include "main.h"
#include "filters.h"
#include "imu.h"
#include "AHRS.h"

SPI_HandleTypeDef *IMU_hspi;
GPIO_TypeDef *IMU_CS_GPIOx;
uint16_t IMU_CS_pin;

biquadFilter_t gxbi,gybi,gzbi;
uint8_t IMU_data_adress = 0x3B | 0x80;









void calabrate_acc_gyr (imu_t * imu)
{
	uint16_t cal_samples = 1000;


	imu->accx_offset = 0;
	imu->accy_offset = 0;
	imu->accz_offset = 0;
	imu->gyrx_offset = 0;
	imu->gyry_offset = 0;
	imu->gyrz_offset = 0;

for (uint8_t i=0; i < 20; i++)
{
	IMU_read_raw(imu);
	HAL_Delay(1);
}

	for (uint16_t cal_buff_ptr; cal_buff_ptr < cal_samples; cal_buff_ptr++)
	{
		IMU_read_raw(imu);

		imu->accx_offset += (float)imu->accx/cal_samples;
		imu->accy_offset += (float)imu->accy/cal_samples;
		imu->accz_offset += ((float)(imu->accz - ACC_1G))/cal_samples;
		imu->gyrx_offset += (float)imu->gyrx/cal_samples;
		imu->gyry_offset += (float)imu->gyry/cal_samples;
		imu->gyrz_offset += (float)imu->gyrz/cal_samples;
		//HAL_Delay(1);
		printf("%f	%f	%f	%f\r\n",imu->accx,imu->accy,imu->accz,imu->accz_offset);
	}




	WriteFlashImuOffsets(0x08040000,imu);
	ReadFlashImuOffsets(0x08040000,imu);
	printf("\r\n\r\n\r\n\r\n");
	printf("%f	%f	%f\r\n",imu->accx_offset,imu->accy_offset,imu->accz_offset);
	printf("%f	%f	%f\r\n",imu->gyrx_offset,imu->gyry_offset,imu->gyrz_offset);
	printf("\r\n\r\n\r\n\r\n");


	//printf("%f\r\n",test1234);

}


void ReadFlashImuOffsets(uint32_t flash_addres, imu_t *data)
{
	data->accx_offset = *(float*)(flash_addres);
	data->accy_offset = *(float*)(flash_addres + 4);
	data->accz_offset = *(float*)(flash_addres + 8);
	data->gyrx_offset = *(float*)(flash_addres + 12);
	data->gyry_offset = *(float*)(flash_addres + 16);
	data->gyrz_offset = *(float*)(flash_addres + 20);
}

void WriteFlashImuOffsets (uint32_t flash_addres,imu_t *data)
{

    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(FLASH_SECTOR_6,VOLTAGE_RANGE_1);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres),*(uint32_t*)&data->accx_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres + 4),*(uint32_t*)&data->accy_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres + 8),*(uint32_t*)&data->accz_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres + 12),*(uint32_t*)&data->gyrx_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres + 16),*(uint32_t*)&data->gyry_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(flash_addres + 20),*(uint32_t*)&data->gyrz_offset);

    HAL_FLASH_Lock();
}







void configure_imu (SPI_HandleTypeDef *hspi123,GPIO_TypeDef *GPIOx123, uint16_t GPIO_Pin123)
{

	ReadFlashImuOffsets(0x08040000,&IMU);


	init_biquad_filter(&gxbi,80.0f,4000.0f);
	init_biquad_filter(&gybi,80.0f,4000.0f);
	init_biquad_filter(&gzbi,80.0f,4000.0f);

	IMU_hspi = hspi123;
	IMU_CS_GPIOx = GPIOx123;
	IMU_CS_pin = GPIO_Pin123;

	IMU_write_bits(0x6A,0x10);
	HAL_Delay(10);


	IMU_write_bits(0x6B,0x80);
	HAL_Delay(100);

	IMU_write_bits(0x68,0x7);
	HAL_Delay(100);

	IMU_write_bits(0x68,0b000000000);
	HAL_Delay(10);

	IMU_write_bits(0x6B,0b000000000);
	HAL_Delay(10);

	IMU_write_bits(0x6A,0x10);
	HAL_Delay(10);

	IMU_write_bits(0x1C,0b00010000);
	HAL_Delay(10);

	IMU_write_bits(0x1B,0b00011000);
	HAL_Delay(100);

	//twoKp = 25;//speed up initial conversion;
	//IMU_read(&IMU);
	//IMU_read(&IMU);
	//q0 = 0;
	//q1 = IMU.accx/2048;
	//q2 = IMU.accx/2048;
	//q3 = IMU.accx/2048;
	//for (uint16_t i=0; i<0xFFF; i++)
	//{
	//IMU_read(&IMU);
	//updateIMU(&IMU);
	//HAL_Delay(1);
	///}




}


void IMU_write_bits(uint8_t adress,uint8_t data)
{
	uint8_t data_tx[2] = {adress,data};

	HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(IMU_hspi, &data_tx[0], 2, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_SET);
}

void IMU_read (imu_t * imu)
{
HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_RESET);
HAL_SPI_TransmitReceive(IMU_hspi, &IMU_data_adress, &IMU_data, 14,100);
HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_SET);

imu->accx = ((int16_t)((IMU_data[0]) + (IMU_data[1]<<8))-imu->accx_offset);
imu->accy = ((int16_t)((IMU_data[2]) + (IMU_data[3]<<8))-imu->accy_offset);
imu->accz = ((int16_t)((IMU_data[4]) + (IMU_data[5]<<8))-imu->accz_offset);
imu->temp  =  (float)((IMU_data[6]) + (IMU_data[7]<<8));
imu->gyrx = biquadFilterApply(&gxbi,(int16_t)((IMU_data[8]) + (IMU_data[9]<<8))-imu->gyrx_offset);
imu->gyry = biquadFilterApply(&gybi,(int16_t)((IMU_data[10]) + (IMU_data[11]<<8))-imu->gyry_offset);
imu->gyrz = biquadFilterApply(&gzbi,(int16_t)((IMU_data[12]) + (IMU_data[13]<<8))-imu->gyrz_offset);
 }

void IMU_read_raw (imu_t * imu)
{
	HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(IMU_hspi, &IMU_data_adress, &IMU_data, 14,100);
	HAL_GPIO_WritePin(IMU_CS_GPIOx, IMU_CS_pin, GPIO_PIN_SET);

	imu->accx = (int16_t)((IMU_data[0]) + (IMU_data[1]<<8));
	imu->accy = (int16_t)((IMU_data[2]) + (IMU_data[3]<<8));
	imu->accz = (int16_t)((IMU_data[4]) + (IMU_data[5]<<8));

	imu->temp  =  (float)((IMU_data[6]) + (IMU_data[7]<<8));

	imu->gyrx = (int16_t)((IMU_data[8]) + (IMU_data[9]<<8));
	imu->gyry = (int16_t)((IMU_data[10]) + (IMU_data[11]<<8));
	imu->gyrz = (int16_t)((IMU_data[12]) + (IMU_data[13]<<8));

}







