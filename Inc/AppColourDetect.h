/*
 *	File name:		AppColourDetect.h
 *	Author:				Er Jui Pin
 *  Date created: 7 Jan 2019
 *
 */

#ifndef APPCOLOURDETECT_H_
#define APPCOLOURDETECT_H_

#include "stm32f3xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define CTRL 0x00
#define CONFIG 0x01

#define CAP_RED 0x06
#define CAP_GREEN 0x07
#define CAP_BLUE 0x08
#define CAP_CLEAR 0x09

#define INT_RED_LO 0x0A
#define INT_RED_HI 0x0B

#define INT_GREEN_LO 0x0C
#define INT_GREEN_HI 0x0D

#define INT_BLUE_LO 0x0E
#define INT_BLUE_HI 0x0F

#define INT_CLEAR_LO 0x10
#define INT_CLEAR_HI 0x11

#define DATA_RED_LO 0x40
#define DATA_RED_HI 0x41

#define DATA_GREEN_LO 0x42
#define DATA_GREEN_HI 0x43

#define DATA_BLUE_LO 0x44
#define DATA_BLUE_HI 0x45

#define DATA_CLEAR_LO 0x46
#define DATA_CLEAR_HI 0x47

#define OFFSET_RED	0x48
#define OFFSET_GREEN	0x49
#define OFFSET_BLUE	0x4A
#define	OFFSET_CLEAR	0x4B

#define COLOUR_SENSOR 0x74

#define CAPACITOR	7	//0x0F
#define GAIN_RED	1180	//525	//1250
#define GAIN_GREEN	1195	//625	//1250
#define GAIN_BLUE	865	//740	//1250
#define GAIN_CLEAR	610	//215

#define CALIBRATE_RED	920
#define CALIBRATE_GREEN	920
#define CALIBRATE_BLUE	400
#define CALIBRATE_CLEAR	980
#define CALIBRATION_ITERATIONS	20

#define NUM_OF_DATA_COLOURSENSOR 5

//extern uint8_t halfSecondFlag, secondFlag, fiveSecondFlag, errorFlag, enterFlag, exitFlag, i, j, counter, autoLogin;
//extern uint8_t menuSelection, receiveOneInput, receiveTwoInputsFirst,receiveTwoInputsSecond, receiveInputArray[NUM_OF_UART_RECEIVED+1], enterPlacement;
//extern uint8_t buffer[100], receiveArray[NUM_OF_UART_RECEIVED+1];

extern void _Error_Handler(char *, int);
extern uint16_t secondCount;

uint8_t Data[8], bufferColourSensor[10], x=0;
int8_t OffsetValues[4];

uint32_t address, value, Red, Green, Blue, Clear, max;
uint32_t CapacitorValue=CAPACITOR, RedGain=GAIN_RED, GreenGain=GAIN_GREEN, BlueGain=GAIN_BLUE, ClearGain=GAIN_CLEAR;
uint32_t RedColourSensorArray[NUM_OF_DATA_COLOURSENSOR], GreenColourSensorArray[NUM_OF_DATA_COLOURSENSOR], BlueColourSensorArray[NUM_OF_DATA_COLOURSENSOR], ClearColourSensorArray[NUM_OF_DATA_COLOURSENSOR];

void SelectColourSensorMenuOnUart(void);
void SelectCapacitorAmount(uint16_t y);
void SetSensorGains(void);
void CollectColourSensorData(void);
void TrimOffset(void);
void CalculateColourSensorData(void);
void PrereadColourSensorData(void);

void StartColourDetectionTest(void);
void SetCapacitorAmount(void);
void SetRedGain(void);
void SetGreenGain(void);
void SetBlueGain(void);
void SetClearGain(void);
void AutoCalibrateSensorGains(void);

#endif /* APPCOLOURDETECT_H_ */
