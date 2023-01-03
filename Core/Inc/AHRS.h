#ifndef AHRS_H
#define AHRS_H

#include "imu.h"
#include "stdbool.h"
#include "filters.h"

#define DEFAULT_SAMPLE_FREQ	4000.0f
void update_pos_z (void);

bool M_FLAG;
bool NEW_BARO;
bool ARMED;
bool LIDAR_IS_VALID;

float baroAltResidual;

float iaccx,iaccy,iaccz;
float iiaccx,iiaccy,iiaccz;
float iiiaccx,iiiaccy,iiiaccz;
float invSqrt(float x);


	void computeAngles();



	void updateIMU(imu_t * data);


	float roll, pitch, yaw, costilt;


	typedef struct fpVector3_s{
	       float x,y,z;
	} fpVector3_t;



	typedef struct {
	    float q0, q1, q2, q3;
	} fpQuaternion_t;





typedef struct BaroStateMatrix_s
{
	float GndPos;
	float Pos;
	float dt;
}BaroStateMatrix_t;

typedef struct StateMatrix_s
{
	float Pos;
	float Vel;

}StateMatrix_t;



#define BARO_LAG  0.05 //s
#define Fast_FREQUENCY  4000 //s
#define DELAY_BUFFER_SIZE 200

float DelayBuffer[500];
uint16_t BuffPtr;
float output;


float test1234;
typedef struct NavAlt_s
{
	float newBaroAlt;
	float BaroAlt;
	float InitBaroAlt;
	float BaroVel;
	float BaroAltLast;
	float accint;
	float accint_delayed;
	float Acc;
	float Pos;
	float Vel;
	float VelEst;
	float PosEst;
	float VelCorr;
	float PosCorr;
	float Acc_feedback;
	float dt;
	float Kg2;
	float Kg;
	pt1Filter_t PosFilter;
	pt1Filter_t VelFilter;
}NavAlt_t;

NavAlt_t NavAlt;


typedef struct AccStateMatrix_s
{
	float Acc;
	float dt;
	float Halfsqdt;
	float Bias;
	float Weight;
	float SqWeight;
}AccStateMatrix_t;

typedef struct NavZ_s
{
	BaroStateMatrix_t Baro;
	AccStateMatrix_t Acc;
	StateMatrix_t Est;
	StateMatrix_t Rusult;
	StateMatrix_t Corr;
}NavZ_t;

NavZ_t NavZ;

float pressureToAltitude(const float pressure);

	fpVector3_t accelBF;

	fpQuaternion_t ref;

#endif
