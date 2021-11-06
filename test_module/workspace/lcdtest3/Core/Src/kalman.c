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

float updateEstimate(struct SimpleKalman* myFilter, float mea)
{

  myFilter->_kalman_gain = myFilter->_err_estimate/(myFilter->_err_estimate + myFilter->_err_measure);
  myFilter->_current_estimate = myFilter->_last_estimate + myFilter->_kalman_gain * (mea - myFilter->_last_estimate);
  myFilter->_err_estimate =  (1.0 - myFilter->_kalman_gain)*myFilter->_err_estimate + fabs(myFilter->_last_estimate-myFilter->_current_estimate)*myFilter->_q;
  myFilter->_last_estimate=myFilter->_current_estimate;

  return myFilter->_current_estimate;
}

