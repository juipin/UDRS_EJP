/*
 *	File name:		uart2.c
 *	Author:				AJAY
 *	Modified by:	Er Jui Pin
 *  Date modified:7 Jan 2019
 *
 */

/**
  ******************************************************************************
  * @file           : uart2.c
  * @brief          : Communitcation using UART2 with PC Hyper-Terminal
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	
#include "main.h"
#include "stm32f3xx_hal.h"
#include "uart2.h"

/* Struct containing Device Selection in the menu */
UDRS_OperationTypedef	APP_JP_operation[] = 
{
	{SetDate, "1: Set Date", 0},
	{SetTime, "2: Set Time", 0},
	{SelectForceSensorMenuOnUart, "3: Force Sensor for Bag Full Detection", 0},
	{SelectPressureSensorMenuOnUart, "4: Pressure Sensor", 0},
	{SelectColourSensorMenuOnUart, "5: Colour Sensor for Ammonia Detection", 0},
	{SelectTopMenuOnUart, "6: Select Top Menu", 0},
	{SetExit, "7: Exit to Top Menu", 0},
	
};

//Reset buffer to store next new data
void ResetBuffer(void)
{
	for(i=0;i<100;i++)
	{
		buffer[i]=0;
		receiveArray[i]=0;
	}
}

/***************************************************************/
//Converts the message into an array
//Transmit array message using HAL_UART_Transmit function(UART)
//Calling ResetBuffer to reset buffer
/***************************************************************/
void API_USR_TransmitMessage(char *message)
{
	snprintf((char *)buffer, sizeof(buffer), "%s", message);
	if(HAL_UART_Transmit(&huart2,buffer, sizeof(buffer),100) !=HAL_OK)
	{
		;
	}
	ResetBuffer();

}
void DeleteLine(void)
{
	for(i=0;i<NUM_OF_UART_RECEIVED;i++)
	{
		buffer[i]=DELETE;
	}
	if(HAL_UART_Transmit(&huart2,buffer,NUM_OF_UART_RECEIVED,100) !=HAL_OK)
	{
		;
	}
}

void AutoLogin(uint8_t x)
{
	if(1 == secondFlag)
	{
		secondFlag=0;
		counter++;

		if((counter == COUNTER_TIMING) && (0 == buffer[0]) && (autoLogin != 0))
		{
			counter=0;
			API_USR_TransmitMessage("\r\nAuto logging...\r\n\r\n");
			receiveArray[j]=x;
			receiveArray[j+1]=ENTER;
			autoLogin=0;
		}
	}
}

/***************************************************************/
//x is the number of variables you want to receive
//The funtion will continue to loop till user press ENTER key(13)
//Sends user's inputs to display
//Problem: only diaplay after pressing ENTER
/***************************************************************/

void ReceiveUart(void)
{
	ResetBuffer();
	enterFlag=0;
	counter=0;
	j=0;


/*	if(1 == x)
	{
		while(0 == enterFlag) //13 is pressing enter
		{

			if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
			{
				receiveArray[j]=buffer[0];
				j++;

				if(j>NUM_OF_UART_RECEIVED)
					j=0;

				DeleteLine();
				HAL_UART_Transmit(&huart2,receiveArray,j,100);
			}

			if(receiveArray[NUM_OF_UART_RECEIVED] != 0)
			{
				ResetBuffer();
				API_USR_TransmitMessage("\r\n\r\nSorry your input is too long...\r\n\r\n");
			}
			AutoLogin(autoLogin);
			for(i=0;i<NUM_OF_UART_RECEIVED;i++)
			{
				if(receiveArray[i]==ENTER)
				{
					enterFlag=1;
					receiveOneInput=receiveArray[i-1];
					ResetBuffer();

				}
			}


		}
	}

	else if(2 == x)
	{
		while(0 == enterFlag) //13 is pressing enter
		{

			if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
			{
				receiveArray[j]=buffer[0];
				j++;

				if(j>NUM_OF_UART_RECEIVED)
					j=0;

				DeleteLine();
				HAL_UART_Transmit(&huart2,receiveArray,j,100);
			}

			if(receiveArray[NUM_OF_UART_RECEIVED] != 0)
			{
				ResetBuffer();
				API_USR_TransmitMessage("\r\n\r\nSorry your input is too long...\r\n\r\n");
			}

			for(i=0;i<NUM_OF_UART_RECEIVED;i++)
			{
				//HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
				if(receiveArray[i]==ENTER)
				{
						//HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
						enterFlag=1;
						receiveTwoInputsFirst=receiveArray[i-2];
						receiveTwoInputsSecond=receiveArray[i-1];
						ResetBuffer();
				}
			}
		}
	}*/

	while(0 == enterFlag) //13 is pressing enter
	{

		if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
		{
			receiveArray[j]=buffer[0];
			j++;

			if(j>NUM_OF_UART_RECEIVED)
				j=0;

			DeleteLine();
			if(HAL_UART_Transmit(&huart2,receiveArray,j,100) !=HAL_OK)
			{
				;
			}
		}else
		{
			;
		}

		if(receiveArray[NUM_OF_UART_RECEIVED] != 0)
		{
			ResetBuffer();
			API_USR_TransmitMessage("\r\n\r\nSorry your input is too long...\r\n\r\n");
		}else
		{
			AutoLogin(autoLogin);
			for(i=0;i<NUM_OF_UART_RECEIVED;i++)
			{
				if(receiveArray[i]==ENTER)
				{
					//If ENTER is the first key (no data enter), take default selection
					if(0 == i)
					{
						counter=0;
						API_USR_TransmitMessage("\r\nAuto logging...\r\n\r\n");
						receiveInputArray[0]=autoLogin; //assume autoLogin has only 1 digit
						receiveInputArray[1]=ENTER;
						autoLogin=0;
						enterPlacement=0;
					}else
					{
						enterPlacement=i-1;
						for(j=enterPlacement;j<NUM_OF_UART_RECEIVED;j--)
						{
							receiveInputArray[j]=receiveArray[j];
						}
					}
						enterFlag=1;
						//receiveOneInput=receiveArray[enterPlacement];
						ResetBuffer();
				}
			}
		}

	}
}

uint32_t ReceiveUartCalculationValue(void)
{
	uint32_t calculateValue;
	ReceiveUart();
	uint8_t x;
	for(x=0;x<NUM_OF_CALCULATION;x++)
	{
		if(0 == receiveInputArray[x])
			receiveInputArray[x]='0';
	}

	calculateValue=(receiveInputArray[enterPlacement]-48);

	if(enterPlacement-1 >= 0 && enterPlacement-1 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-1]-48)*10+calculateValue;

	if(enterPlacement-2 >= 0 && enterPlacement-2 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-2]-48)*100+calculateValue;

	if(enterPlacement-3 >= 0 && enterPlacement-3 <= NUM_OF_CALCULATION)
		calculateValue=(receiveInputArray[enterPlacement-3]-48)*1000+calculateValue;

	if(enterPlacement-4 >= 0 && enterPlacement-4 <= NUM_OF_CALCULATION)
			calculateValue=(receiveInputArray[enterPlacement-4]-48)*10000+calculateValue;

	if(enterPlacement-5 >= 0 && enterPlacement-5 <= NUM_OF_CALCULATION)
				calculateValue=(receiveInputArray[enterPlacement-5]-48)*100000+calculateValue;

	if(calculateValue > 999999)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is invaild...\r\n\r\n");
		ReceiveUartCalculationValue();
	}

	ResetBuffer();



/*	for(x=1;x<NUM_OF_CALCULATION;x++)
	{
		enterPlacement=enterPlacement-1;

		if(enterPlacement >= 0 && enterPlacement < NUM_OF_CALCULATION)
		{
			calculateValue=((receiveInputArray[enterPlacement]-48)*(10*x))+calculateValue;
		}

	}*/



/*	for(x=0;x<(NUM_OF_CALCULATION+1);x++)
	{
		if(0 == receiveInputArray[x])
			receiveInputArray[x]='0';

		if((enterPlacement-x) >= 0)
			{
				calculateValue=((receiveInputArray[enterPlacement-x]-48)*(10*x))+calculateValue;
			}
	}*/

	//return calculateValue= ((receiveInputArray[enterPlacement-3]-48)*1000)+((receiveInputArray[enterPlacement-2]-48)*100)+((receiveInputArray[enterPlacement-1]-48)*10)+(receiveInputArray[enterPlacement]-48);


	return calculateValue;
}

//Displaying default menu
void MainMenu(void)
{
//	API_USR_TransmitMessage("\n\n\n\n#######################################################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n");
//	API_USR_TransmitMessage("######################           ,#####################\r\n");
//	API_USR_TransmitMessage("########################       .#######################\r\n");
//	API_USR_TransmitMessage("##########################   .#########################\r\n");
//	API_USR_TransmitMessage("############         .#####(######.        ,###########\r\n");
//	API_USR_TransmitMessage("############  .,,,.  .(((######(((   ,,,,  .###########\r\n");
//	API_USR_TransmitMessage("#############(  (###(    *####,   .####. .#############\r\n");
//	API_USR_TransmitMessage("###########(..    .(##*  *####,  ###/     ..###########\r\n");
//	API_USR_TransmitMessage("###########,   /##. ##*  *####,  ##/ /##/   *##########\r\n");
//	API_USR_TransmitMessage("##############  .,  ##*  *####,  ##* ,*  .#############\r\n");
//	API_USR_TransmitMessage("##########/   .###  ##*  *####,  ##* (##,    ##########\r\n");
//	API_USR_TransmitMessage("##########,.   (##  ##*  *####,  ##* /###   .(#########\r\n");
//	API_USR_TransmitMessage("#############.      ##*  *####,  ##(      ,#%##########\r\n");
//	API_USR_TransmitMessage("#########,   .###* ,##*  *####,  ###  ###/    (########\r\n");
//	API_USR_TransmitMessage("########(.   /##(  ###*  *####,  ###/ ,###    ,########\r\n");
//	API_USR_TransmitMessage("############.     *#####.      /#####      *###########\r\n");
//	API_USR_TransmitMessage("#######,          /(################/,          #######\r\n");
//	API_USR_TransmitMessage("#######  */////*,,.     ./####*       .,,*////  /######\r\n");
//	API_USR_TransmitMessage("######, ,##############(,      *##############,  ######\r\n");
//	API_USR_TransmitMessage("#####/  .,,,***/(######################//****,,  .#####\r\n");
//	API_USR_TransmitMessage("#####                 ./#######(*                 #####\r\n");
//	API_USR_TransmitMessage("####################*.     ,      ./###################\r\n");
//	API_USR_TransmitMessage("#########################(.  *#########################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n");
//	API_USR_TransmitMessage("#######################################################\r\n\n");


//	API_USR_TransmitMessage("/**********************Main Menu**********************/\r\n");
//	API_USR_TransmitMessage("* 1- Set Time                                         *\r\n");
//	API_USR_TransmitMessage("* 2- Set Date                                         *\r\n");
//	API_USR_TransmitMessage("* 3- Set PSI                                          *\r\n");
//	API_USR_TransmitMessage("* 4- Testing Colour Sensor                            *\r\n");
//	API_USR_TransmitMessage("* 5- Exit                                             *\r\n");
//	API_USR_TransmitMessage("/*****************************************************/\r\n\n");
//	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n");

	API_USR_TransmitMessage("/**********************Main Menu**********************/\r\n");
	API_USR_TransmitMessage("*                                                     *\r\n");
	API_USR_TransmitMessage("* 1- Set Date                                         *\r\n");
	API_USR_TransmitMessage("* 2- Set Time                                         *\r\n");
	API_USR_TransmitMessage("* 3- Select Force Sensor                              *\r\n");
	API_USR_TransmitMessage("* 4- Select Pressure Sensor                           *\r\n");
	API_USR_TransmitMessage("* 5- Select Colour Sensor (default)                   *\r\n");
	API_USR_TransmitMessage("* 6- Return to Main Menu                              *\r\n");
	API_USR_TransmitMessage("* 7- Exit                                             *\r\n");
	API_USR_TransmitMessage("*                                                     *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*         Time: %02d:%02d           Date: %02d/%02d           *\r\n",sTime.Hours,sTime.Minutes,sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("/*****************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}

void SetTime(void)
{
//	API_USR_TransmitMessage("\r\nSetting Time...\r\n");
//	HAL_Delay(500);
//	API_USR_TransmitMessage("Communicating to RTC.\r\n");
//	HAL_Delay(500);
//	API_USR_TransmitMessage("Communicating to RTC..\r\n");
//	HAL_Delay(500);
//	API_USR_TransmitMessage("Communicating to RTC...\r\n");
//	HAL_Delay(500);


//	if(HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY)
//	{
//		API_USR_TransmitMessage("\r\nUnable to communicate with RTC\r\n");
//	}

//	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
//	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BCD);

//	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe time now is %02d:%02d\r\n",sTime.Hours,sTime.Minutes);
//	API_USR_TransmitMessage((char *)buffer);

	//Setting the HOURS

	API_USR_TransmitMessage("\r\n\r\nPlease enter the HOUR now in 24 Hours.(HH)\r\n");
	//ReceiveUart();
	//hour=((receiveTwoInputsFirst-48)*10)+(receiveTwoInputsSecond-48);  //48 to convert ascii num to int
	hour = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nhour = %d\r\n",hour);
	API_USR_TransmitMessage((char *)buffer);
	//if(hour<0 || hour>24)//Checking for invaild inputs
	if(hour>23)//Checking for invaild inputs
	{
		HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetTime();
	}

	ResetBuffer();

	//Setting the MINUTES

	API_USR_TransmitMessage("\r\n\r\nPlease enter the MINUTES now in 24 Hours.(MM)\r\n");
	//ReceiveUart();
	//min=((receiveInputArray[enterPlacement-1]-48)*10)+(receiveInputArray[enterPlacement]-48);  //48 to convert ascii num to int
	min = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nmin = %d\r\n",min);
	API_USR_TransmitMessage((char *)buffer);
	//if(min<0 || min>60)//Checking for invaild inputs
	if(min>59)//Checking for invaild inputs
	{
		HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetTime();
	}
	sTime.Hours=hour;
	sTime.Minutes=min;
	if(HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BCD) != HAL_OK)
	{
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
		API_USR_TransmitMessage("RESTARTING...\r\n");
		SetTime();
	}

	API_USR_TransmitMessage("Setting the Time now...\r\n");
	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe time now is %02d:%02d\r\n",sTime.Hours,sTime.Minutes);
	API_USR_TransmitMessage((char *)buffer);
	HAL_Delay(500);
	API_USR_TransmitMessage("\r\nYour time is all set!\r\n\r\n");
	HAL_Delay(500);


	ResetBuffer();
	MainMenu();
}

void SetDate(void)
{
	API_USR_TransmitMessage("\r\nSetting Date...\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC.\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC..\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Communicating to RTC...\r\n");
	HAL_Delay(500);

	if(HAL_RTC_GetState(&hrtc) != HAL_RTC_STATE_READY )
	{
		API_USR_TransmitMessage("\r\nUnable to communicate with RTC\r\n");
	}

	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BCD);

	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe date today is %02d/%02d\r\n",sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);

	//Setting the month

	API_USR_TransmitMessage("\r\n\r\nPlease enter the MONTH now.(MM)\r\n");
	//ReceiveUart();
	//month=((receiveInputArray[enterPlacement-1]-48)*10)+(receiveInputArray[enterPlacement]-48);  //48 to convert ASCII num to int
	month = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\nmonth = %d\r\n",month);
	API_USR_TransmitMessage((char *)buffer);
	//if(month<0 || month>12)//Checking for invalid inputs
	if(month>12)//Checking for invalid inputs
	{
		HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetDate();
	}

	ResetBuffer();

	//Setting your date

	API_USR_TransmitMessage("\r\n\r\nPlease enter the DATE now.(DD)\r\n");
	//ReceiveUart();
	//date=((receiveTwoInputsFirst-48)*10)+(receiveTwoInputsSecond-48);  //48 to convert ASCII num to int
	date = ReceiveUartCalculationValue();
	snprintf((char *)buffer,sizeof(buffer),"\r\ndate = %d\r\n",date);
	API_USR_TransmitMessage((char *)buffer);
	//if(date<0 || date>31)//Checking for invalid inputs
	if(date>31)//Checking for invalid inputs
	{
		HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
		API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n");
		SetDate();
	}
	sDate.Month=month;
	sDate.Date=date;
	if(HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BCD) != HAL_OK)
	{
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
		API_USR_TransmitMessage("RESTARTING...\r\n");
		SetTime();
	}
	API_USR_TransmitMessage("Setting the DATE now...\r\n");
	snprintf((char *)buffer,sizeof(buffer),"\r\n\r\nThe date today is %02d/%02d\r\n",sDate.Date,sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	HAL_Delay(500);
	API_USR_TransmitMessage("\r\nYour date is all set!\r\n\r\n");
	HAL_Delay(500);

	ResetBuffer();
	MainMenu();
}

void SetPSI(void)
{
	API_USR_TransmitMessage("\r\nSetting PSI...\r\n");

	API_USR_TransmitMessage("\r\n\r\nPlease enter the The PSI rating now.(NN)\r\n");
	//ReceiveUart(2);

	MainMenu();
}

void SetExit(void)
{
	API_USR_TransmitMessage("Exit.\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Exit..\r\n");
	HAL_Delay(500);
	API_USR_TransmitMessage("Exit...\r\n");
	HAL_Delay(500);

	exitFlag=1;
}

void SetColourSensor(void)
{

	SelectColourSensorMenuOnUart();

}

void SettingBluetooth(void)
{
	//SelectionBluetoothMenu();
/*	//PA8 is connected to the STATE Pin of HC-05
	if(1 == HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	{
		API_USR_TransmitMessage("\r\nYour bluetooth is detected!\r\n\r\n");
		HAL_Delay(500);
		SelectionBluetoothMenu();
	}

	else
	{
		API_USR_TransmitMessage("\r\nSorry please check the connection of your bluetooth module\r\n\r\n");
		API_USR_TransmitMessage("\r\nPlease communicate through your bluetooth\r\n");
		SelectTopMenuOnUart();
	}*/

}

void LoadCellMode(void)
{
	API_USR_TransmitMessage("\r\nEntering Load Cell Mode...\r\n");
	//SelectionLoadCellModeMenu();
}

void SelectForceSensorMenuOnUart(void)
{
	;
}

void SelectPressureSensorMenuOnUart(void)
{
	;
}

void SelectTopMenuOnUart(void)
{
	MainMenu();
	exitFlag=0;
	autoLogin='5';
/*	
	while(exitFlag==0)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement];
		switch(menuSelection)
		{
			case ('1'):
				SetTime();
				break;
			case ('2'):
				SetDate();
				break;
			case ('3'):
				SelectForceSensorMenuOnUart();
				break;
			case ('4'):
				SelectPressureSensorMenuOnUart();
				break;
			case ('5'):
				SelectColourSensorMenuOnUart();
				break;
			case ('6'):
				SelectTopMenuOnUart();
				break;
			case ('7'):
				SetExit();
				break;
			default:
				if(menuSelection!='1' || menuSelection!='2' || menuSelection!='3' || menuSelection!='4' || menuSelection!='5'|| menuSelection!='6'|| menuSelection!='7')
				{
					HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
					API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n\r\n\r\n\r\n");
				}
				SelectTopMenuOnUart();
				break;
		}
	}
*/


	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement];
		snprintf((char *)buffer,sizeof(buffer),"enterPlacement: %2d\r\n",enterPlacement);
		API_USR_TransmitMessage((char *)buffer);
		snprintf((char *)buffer,sizeof(buffer),"menuSelection: %2d\r\n",menuSelection);
		API_USR_TransmitMessage((char *)buffer);
		
		menuSelection = menuSelection - 48; // to get integer value from ASCII
		//menuSelection = ReceiveUartCalculationValue();
		//snprintf((char *)buffer,sizeof(buffer),"menuSelection: %2d\r\n",menuSelection);
		//API_USR_TransmitMessage((char *)buffer);
		menuSelection--; //index starts from zero
		
		if(menuSelection >= COUNT_OF_OPERATION(APP_JP_operation)) //check range
		{
			HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
			API_USR_TransmitMessage("\r\nyour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectTopMenuOnUart();
		}else
		{
			APP_JP_operation[menuSelection].OpFunc();
		}
	}
}

