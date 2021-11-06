/*
 * mpu6050.c
 *
 *  Created on: Oct 17, 2021
 *      Author: LENOVO-PC
 */

#define acc_scale 2
#define gyro_scale 500
#define acc_prescaler ((65536.0)/(acc_scale*2))
#define gyro_prescaler (65536.0/(gyro_scale*2))
#include "mpu6050.h"
#include "kalman.h"
#include "math.h"

static struct KalmanMPU myKalman = { 0 };

void MPU_Init(void) {
	uint8_t data_sent = 0;
	//KalmanInit(float mea_e[6], float est_e[6], float q[6]);
	float mea_e[6] = { 0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
	float est_e[6] = { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 };
	float q[6] = { 0.005, 0.005, 0.005, 0.002, 0.002, 0.002};
	myKalman = KalmanInit(mea_e, est_e, q);
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, PWR_MGMT_1_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);

	data_sent = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, SMPLRT_DIV_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);

	data_sent = 3 << 3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, ACCEL_CONFIG_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);

	data_sent = 1 << 3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, GYRO_CONFIG_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);
}

void MPU_Read_Data(MPU_DATA *Mpu_data) {
	uint8_t Rec[14];
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, Rec, 14, 1000);

	int16_t Accel_X_RAW = (int16_t) ((int16_t) Rec[0] << 8 | Rec[1]);
	int16_t Accel_Y_RAW = (int16_t) ((int16_t) Rec[2] << 8 | Rec[3]);
	int16_t Accel_Z_RAW = (int16_t) ((int16_t) Rec[4] << 8 | Rec[5]);

	int16_t Gyro_X_RAW = (int16_t) ((int16_t) Rec[8] << 8 | Rec[9]);
	int16_t Gyro_Y_RAW = (int16_t) ((int16_t) Rec[10] << 8 | Rec[11]);
	int16_t Gyro_Z_RAW = (int16_t) ((int16_t) Rec[12] << 8 | Rec[13]);

	Mpu_data->Accel_x = Accel_X_RAW / 2048.0;
	Mpu_data->Accel_y = Accel_Y_RAW / 2048.0;
	Mpu_data->Accel_z = Accel_Z_RAW / 2048.0;

	Mpu_data->Gyro_x = (Gyro_X_RAW / gyro_prescaler) * degtopi;
	Mpu_data->Gyro_y = (Gyro_Y_RAW / gyro_prescaler) * degtopi;
	Mpu_data->Gyro_z = (Gyro_Z_RAW / gyro_prescaler) * degtopi;
	return;
}

void MPU_Read_Kalman(MPU_DATA *Mpu_data) {
	MPU_Read_Data(Mpu_data);
	*Mpu_data = updateEstimateMPU(&myKalman, *Mpu_data);
	return;
}
float absolute(float a){
	return (a>0) ? a : -a;
}
void CalculateAngle(MPU_DATA *data, Angle *angle){
	float temp = atan(data->Accel_y/data->Accel_z);
	angle->Angle = (absolute(temp)*180)/M_PI;
	angle->Direction = temp > 0 ? FORWARD : BACKWARD;
}
