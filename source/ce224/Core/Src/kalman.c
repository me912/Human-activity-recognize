/*
 * kalman.c
 *
 *  Created on: 11 thg 10, 2021
 *      Author: LENOVO-PC
 */
#include "kalman.h"
struct SimpleKalman SimpleKalmanInit(float mea_e, float est_e, float q)
{
struct SimpleKalman myFilter;

  myFilter._err_measure=mea_e;
  myFilter._err_estimate=est_e;
  myFilter._q = q;
  return myFilter;
}

struct KalmanMPU	KalmanInit(float mea_e[6], float est_e[6], float q[6]){
	struct KalmanMPU myFilter;
	for(int i=0; i<6; i++){
		myFilter.ALL[i] = SimpleKalmanInit(mea_e[i], est_e[i], q[i]);
	}
	return myFilter;
}

float updateEstimate(struct SimpleKalman* myFilter, float mea)
{

  myFilter->_kalman_gain = myFilter->_err_estimate/(myFilter->_err_estimate + myFilter->_err_measure);
  myFilter->_current_estimate = myFilter->_last_estimate + myFilter->_kalman_gain * (mea - myFilter->_last_estimate);
  myFilter->_err_estimate =  (1.0 - myFilter->_kalman_gain)*myFilter->_err_estimate + fabs(myFilter->_last_estimate-myFilter->_current_estimate)*myFilter->_q;
  myFilter->_last_estimate=myFilter->_current_estimate;

  return myFilter->_current_estimate;
}
MPU_DATA updateEstimateMPU(struct KalmanMPU* myFilter, MPU_DATA mea){
		MPU_DATA result;
		result.Accel_x = updateEstimate(&myFilter->ALL[0], mea.Accel_x);
		result.Accel_y = updateEstimate(&myFilter->ALL[1], mea.Accel_y);
		result.Accel_z = updateEstimate(&myFilter->ALL[2], mea.Accel_z);
		result.Gyro_x = updateEstimate(&myFilter->ALL[3], mea.Gyro_x);
		result.Gyro_y = updateEstimate(&myFilter->ALL[4], mea.Gyro_y);
		result.Gyro_z = updateEstimate(&myFilter->ALL[5], mea.Gyro_z);
		return result;

}

