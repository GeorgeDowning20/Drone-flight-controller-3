#include "main.h"
#include "baro.h"
#include "opticalflow.h"
#include "math.h"
#include "filters.h"
#include "AHRS.h"
#include "AERS.h"
#include "stdbool.h"
#include "ibus.h"
#include "imu.h"

float db_alt,db_altcomp;
float atm_derv;
uint32_t tickdt,tickt1,tickt2;
float lidarAGL,lidar2,dl;

float K_gain = 0.04;
float baro_dift;
float a1;
float baro_alt;
uint16_t lidar_trust;



float pressure_difference;



void update_AERS (void)
{
	//tickt1 =  HAL_GetTick();
	//tickdt = tickt1 - tickt2;  //if overflow rusult invalid anyway

	read_baro();

	db = 0.80*db + 0.20 * ((pressure - b1));

	if ((pressure - b1) > 200){
		db = 0;
	}

	db_alt = -80*db;
	baro_alt += db_alt;


	lidarAGL =(float)opticalflow.lidar.distance * costilt;
	lidar_trust += 0.5*(opticalflow.lidar.quality-lidar_trust);

	if ((lidar_trust<2000)&& (costilt > 0.8)) {

		if (lidar_trust<250 || alt_est<400)
		{
			l_factor = 1;
			alt_est += 0.10*(lidarAGL - alt_est);
		}
		else
		{
			l_factor = 0.05;
			alt_est += db_alt + K_gain*(lidarAGL - alt_est);
		}
	}

	else{
		alt_est += db_alt;
	}


//printf("%f\t%f\t%f\t%f\t%f\r\n",db_altcomp,alt_est,1000*baro_dift,baro_alt,lidarAGL);

	//printf("%f\t%f\t%f\t%f\r\n",alt,baro_alt,1000*d_alt,lidarAGL);







	//printf("%i\r\n",alt);
	//printf("%f	%f	%f	\r\n",roll,pitch,yaw);
 //printf("%f	%f	%f\r\n",iiaccx,iiaccy,iiaccz);
	//printf("%f	%f	%f	\r\n",accelBF.x,accelBF.y,accelBF.z);
	//printf("%f	%f	%f	",IMU.accx,IMU.accy,IMU.accz);
	//printf("%f	%f	%f\r\n",IMU.gyrx,IMU.gyry,IMU.gyrz);

HAL_Delay(10);
a1 = alt_est;
lidar2= lidarAGL;
	tickt2 = tickt1;
	b1 = pressure;
}


