/*
 * pid.h
 *
 *  Created on: 1 thg 11, 2021
 *      Author: LENOVO-PC
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "mpu6050.h"
#define K_P 10
#define K_I 0.00002
#define K_D 2

float PID(Angle *angle);


#endif /* INC_PID_H_ */
