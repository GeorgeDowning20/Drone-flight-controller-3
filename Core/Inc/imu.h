
#ifndef IMU_H
#define IMU_H

uint8_t IMU_data[14];

#define ACC_1G (-65535/16)


float accx,accy,accz;
float gyrx,gyry,gyrz;

typedef struct imu_s
{
	float accx_offset;
	float accy_offset;
	float accz_offset;
	float gyrx_offset;
	float gyry_offset;
	float gyrz_offset;

	float accx,accy,accz;
	float gyrx,gyry,gyrz;
	int16_t temp;
}imu_t;


imu_t IMU;


void IMU_read (imu_t * imu);
void IMU_read_raw (imu_t * imu);
void IMU_write_bits(uint8_t adress,uint8_t data);
void configure_imu (SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void WriteFlashImuOffsets(uint32_t flash_addres, imu_t *data);
void ReadFlashImuOffsets (uint32_t flash_addres,imu_t *data);

#endif
