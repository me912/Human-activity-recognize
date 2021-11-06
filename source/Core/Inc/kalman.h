/*
 * kalman.h
 *
 *  Created on: 11 thg 10, 2021
 *      Author: LENOVO-PC
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <math.h>
#include "mpu6050.h"

struct SimpleKalman{
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
};
struct KalmanMPU{
	struct SimpleKalman ALL[6];
};

struct SimpleKalman SimpleKalmanInit(float mea_e, float est_e, float q);
struct KalmanMPU	KalmanInit(float mea_e[6], float est_e[6], float q[6]);
float updateEstimate(struct SimpleKalman* myFilter, float mea);
MPU_DATA updateEstimateMPU(struct KalmanMPU* myFilter, MPU_DATA mea);

#endif /* INC_KALMAN_H_ */
