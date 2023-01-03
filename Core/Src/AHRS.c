#include "main.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "AHRS.h"
#include "IMU.h"
#include "Baro.h"



float twoKp = 10;// 2 * proportional gain (Kp)
float twoKi = 0.03;

	float q0 = 0.0f;
	float q1 = 0.0f;
	float q2 = 1.0f;
	float q3 = 0.0f;
	float integralFBx = 0.0f;
	float integralFBy = 0.0f;

	float integralFBz = 0.0f;
	float invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;



	void GetAccNEU (void);
	//updateIMU(imu_t * data);


void updateIMU(imu_t * data)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float gx, gy, gz, ax, ay, az;
	float integralFBx,integralFBy,integralFBz;


	ax = data->accx / 4096;
	ay = data->accy / 4096;
	az = data->accz / 4096;

		accelBF.x = ax;
		accelBF.y = ay;
		accelBF.z = az;

		gx = (data->gyrx/16.38375f)*0.0349066;
		gy = (data->gyry/16.38375f)*0.0349066;
		gz = (data->gyrz/16.38375f)*0.0349066;






	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);


		//integralFBx += twoKi * halfex * 0.00025;
		//integralFBy += twoKi * halfey * 0.00025;
		//integralFBz += twoKi * halfez * 0.00025;
		//gx += integralFBx;	// apply integral feedback
		//gy += integralFBy;
		//gz += integralFBz;


		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}


	gx *= (0.5f * invSampleFreq);
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	ref.q0 = q0;
	ref.q1 = q1;
	ref.q2 = q2;
	ref.q3 = q3;

	computeAngles();
	GetAccNEU();
	//PredictionZ();
	//update_pos_z();



	getZ(&NavAlt);




	// iaccx = accelBF.x + 0.8*iaccx;
	//iaccy = accelBF.y + 0.8*iaccy;
	//iaccz = ((accelBF.z-1) + 0.8*(iaccz));


	// iiaccx = iaccx + 0.999*(iiaccx);
	//iiaccy = iaccy+ 0.999*(iiaccy);
	//iiaccz = (iaccz+ 0.999*(iiaccz));
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void computeAngles(void)
{
	roll = atan2f(q0*q1 + q2*q3, -(0.5f - q1*q1 - q2*q2));
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	costilt =  -(1.0f - 2.0f * q1*q1 - 2.0f * q2*q2);
}


static inline fpQuaternion_t * quaternionConjugate(fpQuaternion_t * result, const fpQuaternion_t * q)
{
    result->q0 =  q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;

    return result;
}

static inline fpQuaternion_t * quaternionMultiply(fpQuaternion_t * result, const fpQuaternion_t * a, const fpQuaternion_t * b)
{
  fpQuaternion_t p;

  p.q0 = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
  p.q1 = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
  p.q2 = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
  p.q3 = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;

  *result = p;
  return result;
}


void GetAccNEU (void)
{
    fpQuaternion_t vectQuat, refConj;

    vectQuat.q0 = 0;
    vectQuat.q1 = accelBF.x;
    vectQuat.q2 = accelBF.y;
    vectQuat.q3 = accelBF.z;

    quaternionConjugate(&refConj, &ref);
    quaternionMultiply(&vectQuat, &ref, &vectQuat);
    quaternionMultiply(&vectQuat, &vectQuat, &refConj);

    accelBF.x = vectQuat.q1;
    accelBF.y = vectQuat.q2;
    accelBF.z = vectQuat.q3;

}


void PredictionZ (void)
{

	NavZ.Acc.Acc = (accelBF.z - 1 - NavZ.Acc.Bias) * 9.8 * 100;
	NavZ.Est.Pos += NavZ.Est.Vel * NavZ.Acc.dt + (NavZ.Acc.Acc) * NavZ.Acc.Halfsqdt * NavZ.Acc.Weight;   //SUVAT
	NavZ.Est.Vel += NavZ.Acc.Acc * NavZ.Acc.dt * NavZ.Acc.SqWeight;
}




float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

void CorrectionBaroZ (void)
{
	if (NEW_BARO)
	{

		static float initialBaroAltitudeOffset = 0.0f;

   		 float newBaroAlt = pressureToAltitude(pressure);

    	NavZ.Baro.Pos += (newBaroAlt - initialBaroAltitudeOffset);

		if (!M_FLAG)
		{
			initialBaroAltitudeOffset = newBaroAlt;
			NavZ.Est.Pos = 0;
			NavZ.Baro.GndPos = NavZ.Est.Pos;
		}
		else

		{
			//bool GroundEffect = ARMED && ((AGL < 200) && LIDAR_IS_VALID);

			//const float baroAltResidual = (GroundEffect ? NavZ.Baro.GndPos : NavZ.Baro.Pos) - NavZ.Est.Pos;
			baroAltResidual = NavZ.Baro.Pos - NavZ.Est.Pos;
		    NavZ.Corr.Pos = baroAltResidual * 0.35 * NavZ.Baro.dt;
       	    NavZ.Corr.Vel = baroAltResidual * 0.5916 * NavZ.Baro.dt;

			//if (!GroundEffect)
			//{

       		//}
		}
		NavZ.Acc.Bias = -(baroAltResidual * 1 * 0.01 * NavZ.Baro.dt);
		NEW_BARO = 0;
	}
}
float DelayComp (float input);
void getZ (NavAlt_t *Z)
{


   		if (NEW_BARO)
			{
   			Z->newBaroAlt = pressureToAltitude(pressure);
   			Z->BaroAlt =  Z->newBaroAlt - Z->InitBaroAlt;
   			Z->BaroVel += 1*((Z->BaroAlt - Z->BaroAltLast)*32 - Z->BaroVel);
			Z->BaroAltLast = Z->BaroAlt;
			NEW_BARO = 0;
			}

   		Z->Acc = ((accelBF.z - 1 + Z->Acc_feedback))*9800;
   		Z->dt = 1.0f/4000.0f;
   		Z->Kg = 0.003;
   		Z->Kg2 = 0.002;
		if (!M_FLAG)
		{
			Z->InitBaroAlt = Z->newBaroAlt;
			Z->Pos = 0;
			Z->Vel = 0;
			Z->Acc_feedback += 0.05 * (-(accelBF.z - 1) - Z->Acc_feedback);
		}
		else
		{
			//predict correct filter update Vel
			Z->VelEst = Z->Vel + Z->Acc * Z->dt;
			Z->accint = 0.8*(Z->accint) + Z->Acc * Z->dt;


			Z->VelCorr = Z->Kg2*(Z->BaroVel - Z->Vel);


			Z->accint_delayed = (Z->Kg2*(Z->BaroVel - DelayComp(Z->Vel))) ;

			//Z->VelCorr = -Z->VelCorr +Z->accint_delayed;

			//Z->Vel = pt1Filter(&Z->PosFilter,(Z->VelEst + Z->VelCorr),0.8);
			Z->Vel = Z->VelEst + Z->VelCorr;
			//test1234 = DelayComp(Z->Vel);

			//predict correct filter update Vel
			Z->PosEst = Z->Pos + (Z->Vel * Z->dt) +  (Z->Acc * Z->dt*Z->dt * 0.5);
			Z->PosCorr = Z->Kg*(Z->BaroAlt - Z->Pos);
			//Z->Pos += pt1Filter(&Z->PosFilter,(Z->PosEst + Z->PosCorr),0.8);
			Z->Pos = Z->PosEst + Z->PosCorr;


			//Z->Acc_feedback += Z->VelCorr * (Z->dt) * 0.05;



			//Z->Vel += (Z->Acc * Z->dt) + Z->VelCorr);
			//Z->Pos += (Z->Vel * Z->dt) +  (Z->Acc * ((Z->dt)*(Z->dt)) * 0.5 ) + Z->PosCorr;






		}
	//	(Z->Acc * (sqrtf(Z->dt) / 2))
}




float float_overflow (float num, uint16_t maximum, uint16_t minimum);

float DelayComp (float input)
{


	//output = DelayBuffer[BuffPtr];
	DelayBuffer[BuffPtr] = input;
	BuffPtr++;
	BuffPtr = BuffPtr == 500? 0 : BuffPtr;
	float delaycompsmaple = 1*(input - DelayBuffer[BuffPtr]);
	output = input + delaycompsmaple;
	return output;
}

float float_overflow (float num, uint16_t maximum, uint16_t minimum)
{
if (num < minimum) {num = maximum +(minimum + num);}
	else if (num > maximum){num = minimum +(num - maximum);}
return num;
}


void update_pos_z (void)
{
	PredictionZ();
	CorrectionBaroZ();
	NavZ.Rusult.Pos = NavZ.Est.Pos + NavZ.Corr.Pos;
	NavZ.Rusult.Vel = NavZ.Est.Vel + NavZ.Corr.Vel;
}

void InitKalmanZ (NavZ_t * Init)
{
	Init->Acc.dt = 1.0f/4000.0f;
	Init->Acc.Halfsqdt = (NavZ.Acc.dt*NavZ.Acc.dt)/2.0f;
	Init->Acc.Weight = 1.0f;
	Init->Acc.SqWeight = 1.0f;
	Init->Acc.Weight = 1.0f;
	Init->Acc.Bias = 0;
	Init->Baro.dt = 1.0f/100.0f;
}
