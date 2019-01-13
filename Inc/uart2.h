/*
 *	File name:		uart2.h
 *	Author:				AJAY
 *	Modified by:	Er Jui Pin
 *  Date modified:7 Jan 2019
 *
 */

#ifndef UART2_H_
#define UART2_H_

#include "stm32f3xx_hal.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

void ResetBuffer(void);
void API_USR_TransmitMessage(char *message);
void AutoLogin(uint8_t x);
void ReceiveUart(void);
void MainMenu(void);
void SetTime(void);
void SetDate(void);
void SetPSI(void);
void SetExit(void);
void SelectTopMenuOnUart(void);
void SelectForceSensorMenuOnUart(void);
void SelectPressureSensorMenuOnUart(void);
uint32_t ReceiveUartCalculationValue(void);

//extern uint8_t halfSecondFlag, secondFlag, fiveSecondFlag, errorFlag, enterFlag,exitFlag, i, j, counter, autoLogin;
//extern uint8_t menuSelection, receiveOneInput, receiveTwoInputsFirst,receiveTwoInputsSecond, receiveInputArray[NUM_OF_UART_RECEIVED+1], enterPlacement;
//extern uint8_t buffer[100], receiveArray[NUM_OF_UART_RECEIVED+1];

extern uint8_t month, date, hour, min;

extern void SelectColourSensorMenuOnUart(void);

#endif /* UART2_H_ */
