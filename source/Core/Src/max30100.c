/*
 * max30100.c
 *
 *  Created on: 2 thg 10, 2021
 *      Author: LENOVO-PC
 */


/* An STM32 HAL library written for the the MAX30100 pulse oximeter and heart rate sensor. */
/* Libraries by @eepj www.github.com/eepj */
#include "max30100.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#ifdef __cplusplus
extern "C"{
#endif

I2C_HandleTypeDef *_max30100_ui2c;
UART_HandleTypeDef *_max30100_uuart;
uint8_t _max30100_it_byte = 0x00;
uint8_t _max30100_mode;
uint8_t _max30100_mode_prev;
uint16_t _max30100_ir_sample[16];
uint16_t _max30100_red_sample[16];
uint8_t _max30100_ir_current;
uint8_t _max30100_red_current;
uint8_t _max30100_ir_current_prev;
uint8_t _max30100_red_current_prev;

float _max30100_ir_dcremoval[16];
float _max30100_red_dcremoval[16];
float _max30100_ir_meandiff[16];
float _max30100_red_meandiff[16];
float _max30100_ir_butterworth[16];
float _max30100_red_butterworth[16];
float currentBPM;

void MAX30100_Init(I2C_HandleTypeDef *ui2c, UART_HandleTypeDef *uuart){
	_max30100_ui2c = ui2c;
	_max30100_uuart = uuart;
	MAX30100_Stop();
	MAX30100_ClearFIFO();
}

uint8_t MAX30100_ReadReg(uint8_t regAddr){
	uint8_t reg = regAddr, result;
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, &reg, 1, MAX30100_TIMEOUT);
	HAL_I2C_Master_Receive(_max30100_ui2c, MAX30100_I2C_ADDR, &result, 1, MAX30100_TIMEOUT);
	return result;
}

void MAX30100_WriteReg(uint8_t regAddr, uint8_t byte){
	uint8_t reg[2] = { regAddr, byte };
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, reg, 2, MAX30100_TIMEOUT);
}

void MAX30100_EnableInterrupt(uint8_t a_full, uint8_t tmp_rdy, uint8_t hr_rdy, uint8_t spo2){
	uint8_t itReg = ((a_full & 0x01) << MAX30100_ENB_A_FULL) | ((tmp_rdy & 0x01) << MAX30100_ENB_TMP_RDY) | ((hr_rdy & 0x01) << MAX30100_ENB_HR_RDY) | ((spo2 & 0x01) << MAX30100_ENB_SPO2_RDY);
	MAX30100_WriteReg(MAX30100_INTERRUPT_ENB, itReg);
}

void MAX30100_InterruptHandler(void){
	uint8_t itReg = MAX30100_ReadReg(MAX30100_INTERRUPT);
	if((itReg >> MAX30100_A_FULL) & 0x01){
		MAX30100_ReadFIFO();
		if(_max30100_mode == MAX30100_HRONLY_MODE)
			MAX30100_PlotIrToUART(_max30100_uuart, _max30100_ir_sample, 16);
		else if(_max30100_mode == MAX30100_SPO2_MODE)
			MAX30100_PlotBothToUART(_max30100_uuart, _max30100_red_sample, _max30100_ir_sample, 16);
		MAX30100_SetMode(_max30100_mode);
	}else if((itReg >> MAX30100_TMP_RDY) & 0x01){
		_max30100_temp = MAX30100_ReadTemperature();
		MAX30100_EnableInterrupt(1, 0, 0, 0);
	}else if((itReg >> MAX30100_HR_RDY) & 0x01){

	}else if((itReg >> MAX30100_SPO2_RDY) & 0x01){

	}
}

void MAX30100_SetMode(enum MAX30100_Mode mode){
	_max30100_mode = mode;
	uint8_t modeReg = (MAX30100_ReadReg(MAX30100_MODE_CONFIG) & ~(0x07)) | ((mode << MAX30100_MODE) & 0x07);
	if(mode == MAX30100_SPO2_MODE)
		modeReg |= 0x08;
	else
		modeReg &= ~0x08;
	MAX30100_WriteReg(MAX30100_MODE_CONFIG, modeReg);
	if(_max30100_mode == MAX30100_SPO2_MODE)
		MAX30100_EnableInterrupt(0, 1, 0, 0);
	else if(_max30100_mode == MAX30100_HRONLY_MODE)
		MAX30100_EnableInterrupt(1, 0, 0, 0);
	else
		MAX30100_EnableInterrupt(0, 0, 0, 0);
}
//ok
void MAX30100_SetSpO2SampleRate(enum MAX30100_SpO2SR sr){
	uint8_t spo2Reg = MAX30100_ReadReg(MAX30100_SPO2_CONFIG);
	spo2Reg = ((sr << MAX30100_SPO2_SR) & 0x1c) | (spo2Reg & ~0x1c);
	MAX30100_WriteReg(MAX30100_SPO2_CONFIG, spo2Reg);
}
//ok
void MAX30100_SetLEDPulseWidth(enum MAX30100_LEDPulseWidth pw){
	uint8_t spo2Reg = MAX30100_ReadReg(MAX30100_SPO2_CONFIG);
	spo2Reg = ((pw << MAX30100_LED_PW) & 0x03) | (spo2Reg & ~0x03);
	MAX30100_WriteReg(MAX30100_SPO2_CONFIG, spo2Reg);
}
void MAX30100_SetLEDCurrent(enum MAX30100_LEDCurrent redpa, enum MAX30100_LEDCurrent irpa){
	_max30100_red_current = redpa;
	_max30100_ir_current = irpa;
	MAX30100_WriteReg(MAX30100_LED_CONFIG, (redpa << MAX30100_LED_RED_PA) | irpa);
}

void MAX30100_ClearFIFO(void){
	MAX30100_WriteReg(MAX30100_FIFO_WR_PTR, 0x00);
	MAX30100_WriteReg(MAX30100_FIFO_RD_PTR, 0x00);
	MAX30100_WriteReg(MAX30100_OVF_COUNTER, 0x00);
}

void MAX30100_ReadFIFO(void){
	//uint8_t fifo_wr_ptr = MAX30100_ReadReg(MAX30100_FIFO_WR_PTR);
	//uint8_t fifo_rd_ptr = MAX30100_ReadReg(MAX30100_FIFO_RD_PTR);
	uint8_t num_sample = 64;//(fifo_wr_ptr - fifo_rd_ptr) * 4;
	uint8_t fifo_data[64] = { 0 };
	uint8_t reg = MAX30100_FIFO_DATA;
	HAL_I2C_Master_Transmit(_max30100_ui2c, MAX30100_I2C_ADDR, &reg, 1, MAX30100_TIMEOUT);
	HAL_I2C_Master_Receive(_max30100_ui2c, MAX30100_I2C_ADDR, fifo_data, num_sample, MAX30100_TIMEOUT);
	for(uint8_t i = 0; i < num_sample; i += 4){
		_max30100_ir_sample[i / 4] = (fifo_data[i] << 8) | fifo_data[i + 1];
		_max30100_red_sample[i / 4] = (fifo_data[i + 2] << 8) | fifo_data[i + 3];
	}

}

float MAX30100_ReadTemperature(){
	int8_t tempInt = (int8_t) MAX30100_ReadReg(MAX30100_TMP_INTEGER);
	uint8_t tempFrac = MAX30100_ReadReg(MAX30100_TMP_FRACTION);
	return (tempInt + tempFrac / 16.0);
}

void MAX30100_PlotTemperatureToUART(UART_HandleTypeDef *uuart){
	uint8_t tempInt = _max30100_temp / 1;
	uint8_t tempFrac = (_max30100_temp - tempInt) * 10;
	char data[15];
	sprintf(data, "temp:%d.%d\n", tempInt, tempFrac);
	HAL_UART_Transmit(uuart, data, strlen(data), MAX30100_TIMEOUT);
}

void MAX30100_PlotIrToUART(UART_HandleTypeDef *uuart, uint16_t *samples, uint8_t sampleSize){
	char data[10];
	for(uint8_t i = 0; i< sampleSize; i++){
		sprintf(data, "s:%d\n", samples[i]);
		HAL_UART_Transmit(uuart, data, strlen(data), MAX30100_TIMEOUT);
	}
}

void MAX30100_PlotBothToUART(UART_HandleTypeDef *uuart, uint16_t *samplesRed, uint16_t *samplesIr, uint8_t sampleSize){
	char data[20];
	for(uint8_t i = 0; i< sampleSize; i++){
		sprintf(data, "red:%d\tir:%d\n", samplesRed[i], samplesIr[i]);
		HAL_UART_Transmit(uuart, data, strlen(data), MAX30100_TIMEOUT);
	}
}

void MAX30100_Stop(void){
	_max30100_mode = MAX30100_IDLE_MODE;
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_0_0, MAX30100_LEDCURRENT_0_0);
	MAX30100_WriteReg(MAX30100_INTERRUPT_ENB, 0x00);
}

void MAX30100_Pause(void){
	_max30100_mode_prev = _max30100_mode;
	_max30100_red_current_prev = _max30100_red_current;
	_max30100_ir_current_prev = _max30100_ir_current;
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_0_0, MAX30100_LEDCURRENT_0_0);
	MAX30100_SetMode(MAX30100_IDLE_MODE);
}

void MAX30100_Resume(void){
	MAX30100_SetLEDCurrent(_max30100_red_current_prev, _max30100_ir_current_prev);
	MAX30100_SetMode(_max30100_mode_prev);
}

void Dc_Removal(){
	static float prev_w_ir = 0;
	static float prev_w_red = 0;
	float w;
	for(int i=0; i<16; i++){
		w = _max30100_ir_sample[i] + alpha*prev_w_ir;
		_max30100_ir_dcremoval[i] = w - prev_w_ir;
		prev_w_ir = w;

		w = _max30100_red_sample[i] + alpha*prev_w_red;
		_max30100_red_dcremoval[i] = w - prev_w_red;
		prev_w_red = w;
	}
}

void meanDiff(){
	static float sum_ir = 0;
	static float sum_red = 0;
	static float pre_ir[16] = {0};
	static float pre_red[16] = {0};

	static float count = 0;

	for(int i=0; i<16; i++){
		sum_ir -= pre_ir[i];
		sum_red -= pre_red[i];

		sum_ir += _max30100_ir_dcremoval[i];
		sum_red += _max30100_red_dcremoval[i];

		pre_ir[i] = _max30100_ir_dcremoval[i];
		pre_red[i] = _max30100_red_dcremoval[i];

		if(count < 16) count++;

		_max30100_ir_meandiff[i] = sum_ir/count - pre_ir[i];
		_max30100_red_meandiff[i] = sum_red/count - pre_ir[i];
	}
}

void lowpassfilter(){
		static float prev_ir = 0;
		static float current_ir = 0;

		for(int i=0; i<16; i++){
			prev_ir = current_ir;

			//Fs = 100Hz and Fc = 10Hz
			current_ir = (2.452372752527856026e-1 * _max30100_ir_meandiff[i]) + (0.50952544949442879485 * prev_ir);

			_max30100_ir_butterworth[i] = current_ir + prev_ir;
		}

}
static uint8_t currentPulseDetectorState = 0;
static uint8_t lastBeatThreshold = 0;

uint8_t detectPulse(float sensor_value)
{
  static uint16_t prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;
  static uint8_t bpmIndex=0;
  static uint8_t valuesBPMCount = 0;
  static float valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return 0;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
          if(sensor_value > prev_sensor_value)
          {
            currentBeat = HAL_GetTick();
            lastBeatThreshold = sensor_value;
          }
          else
          {


            uint32_t beatDuration = currentBeat - lastBeat;
            lastBeat = currentBeat;

            float rawBPM = 0;
            if(beatDuration > 0)
              rawBPM = 60000.0 / (float)beatDuration;

            //This method sometimes glitches, it's better to go through whole moving average everytime
            //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
            //valuesBPMSum -= valuesBPM[bpmIndex];
            //valuesBPM[bpmIndex] = rawBPM;
            //valuesBPMSum += valuesBPM[bpmIndex];

            valuesBPM[bpmIndex] = rawBPM;
            float valuesBPMSum = 0;
            for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
            {
              valuesBPMSum += valuesBPM[i];
            }


            bpmIndex++;
            bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

            if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
              valuesBPMCount++;

            currentBPM = valuesBPMSum / valuesBPMCount;

            currentPulseDetectorState = PULSE_TRACE_DOWN;

            return 1;
          }
          break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return 0;
}

uint8_t heart_rate(){
	MAX30100_ReadFIFO();
	Dc_Removal();
	meanDiff();
	lowpassfilter();
	for(int i=0; i<16; i++){
		if(detectPulse(_max30100_ir_butterworth[i])) {
			BPM_value = currentBPM;
			return 1;
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
