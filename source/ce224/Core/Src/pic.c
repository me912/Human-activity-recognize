/*
 * pic.c
 *
 *  Created on: 1 thg 11, 2021
 *      Author: LENOVO-PC
 */

#include "pid.h"



float PID(Angle *angle){
	static float total_err = 0;
	static float last_err = 0;

	total_err += angle->Angle;

	float error = K_P*angle->Angle +
			K_I*(absolute(angle->Angle-last_err)) +
			K_D*total_err;

	last_err = angle->Angle;

	return error;
}


