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

	#define INT_PIN_CFG	0x37
	#define INT_ENABLE 	0x38
	#define INT_STATUS  0x3A
	#define TEMP_OUT_H_REG 0x41
	#define GYRO_XOUT_H_REG 0x43
	#define PWR_MGMT_1_REG 0x6B
	#define WHO_AM_I_REG 0x75

	#define LATCH_INT_EN	(1<<5)
	#define INT_RD_CLEAR  	(1<<4)

	#define DATA_RDY_EN   	(1<<0)




#define sample_rate 50.0
#define cutoff		0.3
#ifndef M_PI
#define M_PI (3.14159265359)
#endif
#define alpha 	((1.0/(2.0*M_PI*cutoff))/(1.0/(2.0*M_PI*cutoff)+1.0/sample_rate))
#include "stm32f4xx.h"
#define degtopi (M_PI/180.0)

typedef struct {
	float Accel_x;
	float Accel_y;
	float Accel_z;
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
} MPU_DATA;

typedef struct {
	float total_acc_x;
	float total_acc_y;
	float total_acc_z;
	float body_acc_x;
	float body_acc_y;
	float body_acc_z;
	float body_gyro_x;
	float body_gyro_y;
	float body_gyro_z;
} Har_InputTypeDef;
I2C_HandleTypeDef hi2c1; // from stm32fxx_it.h
float BPM_value;        // import from max30100.h

void MPU_Init(void);
void MPU_Read_Data(MPU_DATA *data);
void MPU_Read_Kalman(MPU_DATA *data);
void MPU_Read_Data_forHAR(Har_InputTypeDef *data);
float MPU_Mag(MPU_DATA *Mpu_data);
float MPU_mag2(Har_InputTypeDef *mpudata);
#endif /* INC_MPU6050_H_ */
