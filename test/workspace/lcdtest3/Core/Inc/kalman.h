/*
 * kalman.h
 *
 *  Created on: 11 thg 10, 2021
 *      Author: LENOVO-PC
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <math.h>

struct SimpleKalman{
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
};

struct SimpleKalman SimpleKalmanInit(float mea_e, float est_e, float q);
float updateEstimate(struct SimpleKalman* myFilter, float mea);

#endif /* INC_KALMAN_H_ */
