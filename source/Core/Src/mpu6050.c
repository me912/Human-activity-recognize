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
#include "main.h"
#include <math.h>

static struct SimpleKalman myKalman = { 0 };

void MPU_Init(void) {
	uint8_t data_sent = 0;
//	float mea_e[6] = { 0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
//	float est_e[6] = { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 };
//	float q[6] = { 0.005, 0.005, 0.005, 0.002, 0.002, 0.002};
	myKalman = SimpleKalmanInit(1, 0.1, 1);

	//HAL_StatusTypeDef error;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, PWR_MGMT_1_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 5);

//	char* send = "error";
//	if(error == HAL_BUSY) HAL_UART_Transmit(&huart2, send, 5, 100);
	data_sent = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, SMPLRT_DIV_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 5);

	data_sent = 3 << 3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, ACCEL_CONFIG_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 5);

	data_sent = 1 << 3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, GYRO_CONFIG_REG,
	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 5);

//	data_sent = LATCH_INT_EN;
//	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, INT_PIN_CFG,
//	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);
//
//	data_sent = 1<<4;
//	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, INT_ENABLE,
//	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 1000);
}

void MPU_Read_Data(MPU_DATA *Mpu_data) {
	uint8_t Rec[14];
	static MPU_DATA last={0};
	//HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, INT_STATUS, I2C_MEMADD_SIZE_8BIT, Rec, 1, 50);
	HAL_StatusTypeDef check = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, Rec, 14, 2);
	if(check != HAL_OK){
		*Mpu_data = last;
		return;
	}
	int16_t volatile Accel_X_RAW = (int16_t) ((int16_t) Rec[0] << 8 | Rec[1]);
	int16_t volatile Accel_Y_RAW = (int16_t) ((int16_t) Rec[2] << 8 | Rec[3]);
	int16_t volatile Accel_Z_RAW = (int16_t) ((int16_t) Rec[4] << 8 | Rec[5]);

	int16_t volatile Gyro_X_RAW = (int16_t) ((int16_t) Rec[8] << 8 | Rec[9]);
	int16_t volatile Gyro_Y_RAW = (int16_t) ((int16_t) Rec[10] << 8 | Rec[11]);
	int16_t volatile Gyro_Z_RAW = (int16_t) ((int16_t) Rec[12] << 8 | Rec[13]);

	Mpu_data->Accel_x = (Accel_X_RAW / 2048.0 > 3 || Accel_X_RAW / 2048.0 <  -3 ) ? last.Accel_x : Accel_X_RAW / 2048.0;
	Mpu_data->Accel_y = (Accel_Y_RAW / 2048.0 > 3 || Accel_Y_RAW / 2048.0 <  -3 ) ? last.Accel_y : Accel_Y_RAW / 2048.0;
	Mpu_data->Accel_z = (Accel_Z_RAW / 2048.0 > 3 || Accel_Z_RAW / 2048.0 <  -3 ) ? last.Accel_z : Accel_Z_RAW / 2048.0;

	Mpu_data->Gyro_x = (Gyro_X_RAW / gyro_prescaler) * degtopi;
	Mpu_data->Gyro_y = (Gyro_Y_RAW / gyro_prescaler) * degtopi;
	Mpu_data->Gyro_z = (Gyro_Z_RAW / gyro_prescaler) * degtopi;

	last = *Mpu_data;


	return;
}

void MPU_Read_Kalman(MPU_DATA *Mpu_data) {
	MPU_Read_Data(Mpu_data);
	*Mpu_data = updateEstimateMPU(&myKalman, *Mpu_data);
	return;
}
static MPU_DATA data_temp;
void MPU_Read_Data_forHAR(Har_InputTypeDef *data){
	MPU_Read_Data(&data_temp);
	static float x=0,y=0,z=0;
	x = (alpha)*x + (1-alpha)*data_temp.Accel_x;
	y = (alpha)*y + (1-alpha)*data_temp.Accel_y;
	z = (alpha)*z + (1-alpha)*data_temp.Accel_z;

	data->body_acc_x = data_temp.Accel_x - x;
	data->body_acc_y = data_temp.Accel_y - y;
	data->body_acc_z = data_temp.Accel_z - z;

	data->body_gyro_x = data_temp.Gyro_x;
	data->body_gyro_y = data_temp.Gyro_y;
	data->body_gyro_z = data_temp.Gyro_z;

	data->total_acc_x = data_temp.Accel_x;
	data->total_acc_y = data_temp.Accel_y;
	data->total_acc_z = data_temp.Accel_z;
}
float MPU_Mag(MPU_DATA *Mpu_data){
	volatile float temp = Mpu_data->Accel_x*Mpu_data->Accel_x + Mpu_data->Accel_y*Mpu_data->Accel_y + Mpu_data->Accel_z*Mpu_data->Accel_z;
	volatile float temp2 = updateEstimate(&myKalman, sqrtf(temp));
	return temp2;
}
float MPU_mag2(Har_InputTypeDef *mpudata){
	float temp = mpudata->total_acc_x*mpudata->total_acc_x + mpudata->total_acc_y*mpudata->total_acc_y + mpudata->total_acc_z*mpudata->total_acc_z;
	return sqrtf(temp);
}
