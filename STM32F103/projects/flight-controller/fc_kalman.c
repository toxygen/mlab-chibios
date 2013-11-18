/** A simple kalman filter example by Adrian Boeing 
  www.adrianboeing.com 
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "fc_kalman.h"

/* 
 * This is a simple Kalman filter implementation.
 * This filter assumes the current measure is close to the last one is the correct one.
 * This function can be used to even out slow changing precesses, like bearing and altitude.
 * sample values for float Q = 0.022; R = 0.617;
 */
float KALsimple(float z_measured, float x_est_last, float Q, float R)
{
  float K;
  //float P;
  float P_temp;
  //float x_temp_est;
  float x_est;
  //float z_measured; //the 'noisy' value we measured
  //float z_real = 0.5; //the ideal value we wish to measure

	//do a prediction
	//x_temp_est = x_est_last; // very simple, temperature stays stable
	//P_temp = P_last + Q; // Noice in prediction
	P_temp = x_est_last + Q; // Noice in prediction
	//calculate the Kalman gain
	K = P_temp * (1.0/(P_temp + R)); // calculate relevance of meaurement (Kalman Number)
	//measure
	//z_measured = z_real + frand()*0.19; //the real measurement plus noise
	//correct
	x_est = x_est_last + K * (z_measured - x_est_last); // Use the K * difference to correct the measurement
	//P = (1- K) * P_temp; // Calculate new error
	//we have our new system

	//printf("Ideal    position: %6.3f \n",z_real);
	//printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
	//printf("Kalman   position: %6.3f [diff:%.3f] P: %6.3f P_temp: %6.3f K: %6.3f\n",x_est,fabs(z_real - x_est),  P,  P_temp, K);

	//sum_error_kalman += fabs(z_real - x_est);
	//sum_error_measure += fabs(z_real-z_measured);

	return x_est;
}

/*
 * --- Kalman filter module  ----------------------------------------------------------------------------
 * Thanks to http://www.x-firm.com/?page_id=191
 */

#if 0
float Q_angle  =  0.001; //0.001
float Q_gyro   =  0.003;  //0.003
float R_angle  =  0.03;  //0.03

float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
#endif

/*
 * Initialize the Kalman structure.
 */
void KALInit(kalmanStruct *kalStruct)
{
	kalStruct->Q_angle  =  0.001;
	kalStruct->Q_gyro   =  0.003;
	kalStruct->R_angle  =  0.03;

	kalStruct->x_bias = 0;
	kalStruct->P_00 = 0;
	kalStruct->P_01 = 0;
	kalStruct->P_10 = 0;
	kalStruct->P_11 = 0;
}

/*
 * Insert Accelerometer Angle, and Gyroscope rate + time elapsed to get none-drifting angle data.
 * All input should be in Radiants.
 */
float KALCalculate(kalmanStruct *kalStruct, float newAngle, float newRate,int looptime) {
	float dt, y, S;
	float K_0, K_1;

	dt = looptime/1000.0;
	kalStruct->x_angle += dt * (newRate - kalStruct->x_bias);
	kalStruct->P_00 +=  - dt * (kalStruct->P_10 + kalStruct->P_01) + kalStruct->Q_angle * dt;
	kalStruct->P_01 +=  - dt * kalStruct->P_11;
	kalStruct->P_10 +=  - dt * kalStruct->P_11;
	kalStruct->P_11 +=  + kalStruct->Q_gyro * dt;

	y = newAngle - kalStruct->x_angle;
	S = kalStruct->P_00 + kalStruct->R_angle;
	K_0 = kalStruct->P_00 / S;
	K_1 = kalStruct->P_10 / S;

	kalStruct->x_angle +=  K_0 * y;
	kalStruct->x_bias  +=  K_1 * y;
	kalStruct->P_00 -= K_0 * kalStruct->P_00;
	kalStruct->P_01 -= K_0 * kalStruct->P_01;
	kalStruct->P_10 -= K_1 * kalStruct->P_00;
	kalStruct->P_11 -= K_1 * kalStruct->P_01;

	return kalStruct->x_angle;
}
