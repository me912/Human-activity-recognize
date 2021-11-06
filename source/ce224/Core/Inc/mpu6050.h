/*
 * mpu6050.h
 *
 *  Created on: Oct 17, 2021
 *      Author: LENOVO-PC
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#define  MPU_ADDR 0x68
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#ifndef M_PI
#define M_PI (3.14159265359)
#endif
#include "stm32f4xx.h"
#define degtopi (M_PI/180.0)

#define FORWARD 0
#define BACKWARD 1


typedef struct{
	float Accel_x;
	float Accel_y;
	float Accel_z;
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
}MPU_DATA;

typedef struct{
	float Angle;
	uint8_t Direction;
}Angle;

I2C_HandleTypeDef hi2c1; // from stm32fxx_it.h
float BPM_value;        // import from max30100.h

void MPU_Init(void);
void MPU_Read_Data(MPU_DATA *data);
void MPU_Read_Kalman(MPU_DATA *data);
void CalculateAngle(MPU_DATA *data, Angle *angle);
float abosulute(float a);

#endif /* INC_MPU6050_H_ */
