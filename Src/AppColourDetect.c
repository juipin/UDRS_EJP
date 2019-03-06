/*
 *	File name:		AppColourDetect.c
 *	Author:				Er Jui Pin
 *  Date created: 7 Jan 2019
 *
 */

/**
  ******************************************************************************
  * @file           : APP_ColourDetect.c
  * @brief          : Communication with colour sensor
  ******************************************************************************
 */

#include "main.h"
#include "AppColourDetect.h"
//#include "stm32f3xx_hal.h"
#include "uart2.h"

/* Struct containing Device Selection in the menu */
UDRS_OperationTypedef	API_CDT_operation[] = 
{
	{StartColourDetectionTest, "1: Start Colour Detection Test", 0},
	{SetCapacitorAmount, "2: Set Capacitor Amount", 0},
	{SetRedGain, "3: Set Red Channel Gain", 0},
	{SetGreenGain, "4: Set Green Channel Gain", 0},
	{SetBlueGain, "5: Set Blue Channel Gain", 0},
	{SetClearGain, "6: Set Clear Channel Gain", 0},
	{SetDefaultGain, "7: Set Default Channel Gains", 0},
	{AutoCalibrateSensorGains, "8: Auto Calibrate Sensor Gains", 0},
	{SelectTopMenuOnUart, "9: Return To	Top Menu", 0},
	{SetExit, "10: Exit", 0},
	
};

void ReadColourSensorSetting(void)
{
	//Reading the Capacitor value set
	address= CAP_RED;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		CapacitorValue=bufferColourSensor[0];
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Reading the Red Gain
	address= INT_RED_LO;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		RedGain=bufferColourSensor[0];
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_RED_HI;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		RedGain=(bufferColourSensor[0]*256)+RedGain;
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Reading the Green Gain
	address= INT_GREEN_LO;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		GreenGain=bufferColourSensor[0];
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_GREEN_HI;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		GreenGain=(bufferColourSensor[0]*256)+GreenGain;
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Reading the Blue Gain
	address= INT_BLUE_LO;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		BlueGain=bufferColourSensor[0];
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_BLUE_HI;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		BlueGain=(bufferColourSensor[0]*256)+BlueGain;
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Reading the Blue Gain
	address= INT_CLEAR_LO;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		ClearGain=bufferColourSensor[0];
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_CLEAR_HI;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		ClearGain=(bufferColourSensor[0]*256)+ClearGain;
	}
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	snprintf((char *)buffer, sizeof(buffer), "All the Capacitor Values are set at %d!\r\n\r\n", CapacitorValue);
	API_USR_TransmitMessage((char *)buffer);

	snprintf((char *)buffer, sizeof(buffer), "The Red Gain is %d!\r\n\r\n", RedGain);
	API_USR_TransmitMessage((char *)buffer);

	snprintf((char *)buffer, sizeof(buffer), "The Green Gain is %d!\r\n\r\n", GreenGain);
	API_USR_TransmitMessage((char *)buffer);

	snprintf((char *)buffer, sizeof(buffer), "The Blue Gain is %d!\r\n\r\n", BlueGain);
	API_USR_TransmitMessage((char *)buffer);

	snprintf((char *)buffer, sizeof(buffer), "The Clear Gain is %d!\r\n\r\n", ClearGain);
	API_USR_TransmitMessage((char *)buffer);

}

//Selecting Capacitor amount
void SelectCapacitorAmount(uint16_t y)
{

	address= CAP_RED;
	for (x = 0; x <4; x++)
	{

	 bufferColourSensor[0]= y;

	 if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) != HAL_OK)
		 API_USR_TransmitMessage("At least one capacitor value is NOT set! Please select again.\r\n");


	 address++;
	}


/*	address= CAP_RED;
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) == HAL_OK)
	{
		snprintf((char *)buffer, sizeof(buffer), "The Capacitor is set as %d!\r\n\r\n", bufferColourSensor[0]);
		API_USR_TransmitMessage((char *)buffer);
	}*/


}

void SetSensorGains(void)
{
	//Setting the Red Gain
	if(0 == RedGain || RedGain>4095)   //On power-up reset, all Integration Time Slot (or Gain) values are 0, also to take default values
	{
		RedGain=GAIN_RED;
		API_USR_TransmitMessage("\r\n\r\nRed Gain is set as default.\r\n\r\n");
	}

	address= INT_RED_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0]= RedGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is NOT set(LOW)!\r\n");

	address= INT_RED_HI;
	//bufferColourSensor[0]= (RedGain>>8)& 0x00FF;
	bufferColourSensor[0]= (RedGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is NOT set(HIGH)!\r\n");


	//Setting the Green Gain
	if(0 == GreenGain || GreenGain>4095)  //On power-up reset, all Integration Time Slot (or Gain) values are 0, also to take default values
	{
		GreenGain=GAIN_GREEN;
		API_USR_TransmitMessage("\r\n\r\nGreen Gain is set as default.\r\n\r\n");
	}

	address= INT_GREEN_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0]= GreenGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is NOT set(LOW)!\r\n");

	address= INT_GREEN_HI;
	bufferColourSensor[0]= (GreenGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is NOT set(HIGH)!\r\n");


	//Setting the Blue Gain
	if(0 == BlueGain || BlueGain>4095)    //On power-up reset, all Integration Time Slot (or Gain) values are 0, also to take default values
	{
		BlueGain=GAIN_BLUE;
		API_USR_TransmitMessage("\r\n\r\nBlue Gain is set as default.\r\n\r\n");
	}

	address= INT_BLUE_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0]= BlueGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is NOT set(LOW)!\r\n");

	address= INT_BLUE_HI;
	bufferColourSensor[0]= (BlueGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is NOT set(HIGH)!\r\n");



	//Setting the Clear Gain
	if(0 == ClearGain || ClearGain>4095)    //On power-up reset, all Integration Time Slot (or Gain) values are 0, also to take default values
	{
		ClearGain=GAIN_CLEAR;
		API_USR_TransmitMessage("\r\n\r\nClear Gain is set as default.\r\n\r\n");
	}

	address= INT_CLEAR_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0]= ClearGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is NOT set(LOW)!\r\n");

	address= INT_CLEAR_HI;
	bufferColourSensor[0]= (ClearGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)!=HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is NOT set(HIGH)!\r\n");

	//ReadColourSensorSetting();
}


void TrimOffset(void)
{
	//Turn Off LED
	HAL_GPIO_WritePin( ColourSensorLed_GPIO_Port, ColourSensorLed_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	

	//Set Trim offset mode (TOFS)
	//Read the CONFIG register first
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100) != HAL_OK)
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
	bufferColourSensor[0]= bufferColourSensor[0] | 0x01; //Set TOFS bit
	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100) !=HAL_OK)
		API_USR_TransmitMessage("ERROR, TOFS bit is not set properly\r\n");
	
	//Set GOFS bit in CTRL Register to start reading offset values
	bufferColourSensor[0]= 0x02;
  if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, CTRL, 1, bufferColourSensor, 1, 100) !=HAL_OK)
		API_USR_TransmitMessage("ERROR, GOFS bit is not set properly\r\n");

  //Check whether data has been stored in registers
	//bufferColourSensor[0]= 0x02;
  while(bufferColourSensor[0] !=0) //GOFS bit is automatically cleared when result is stored
  {
    if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, CTRL, 1, bufferColourSensor, 1, 100) !=HAL_OK)
			API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
  }
	
	address = OFFSET_RED;
	for(x=0; x<4; x++)
	{
		if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) !=HAL_OK)
			API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
		address++;
		if (bufferColourSensor[0] >= 128) {
			OffsetValues[x]= 128 - bufferColourSensor[0];
		} else
			OffsetValues[x]= bufferColourSensor[0];
		
		snprintf((char *)buffer, sizeof(buffer), "OffsetValues[%d] is %d.\r\n", x, OffsetValues[x]);
		API_USR_TransmitMessage((char *)buffer);
	}
	
	//Set Trim offset mode (TOFS), All digital values of the sensor will automatically trim the offsets
	//Read the CONFIG register first
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100) != HAL_OK)
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
	bufferColourSensor[0]= bufferColourSensor[0] | 0x01; //Set TOFS bit
	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100) !=HAL_OK)
		API_USR_TransmitMessage("ERROR, TOFS bit is not set properly\r\n");
	
/*	
	// Reset Trim offset mode (TOFS)
	if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100) != HAL_OK)
		API_USR_TransmitMessage("ERROR, sorry something went wrong\r\n");
	bufferColourSensor[0]= bufferColourSensor[0] & 0xFE; //Clear TOFS bit
	HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, CONFIG, 1, bufferColourSensor, 1, 100);
*/
}


void CollectColourSensorData(void)
{

  HAL_GPIO_WritePin( ColourSensorLed_GPIO_Port, ColourSensorLed_Pin, GPIO_PIN_SET);                                  //Turn on LED

  bufferColourSensor[0]= 0x01;
  if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, CTRL, 1, bufferColourSensor, 1, 100) !=HAL_OK)	//Send to start reading
	{
    _Error_Handler(__FILE__, __LINE__);
		/* May be caused by system hang at STM side as there is no I2C signals. Reset and re-enable the sensor */
		//MX_I2C1_Init();
		{
			hi2c1.Instance = I2C1;
			hi2c1.Init.Timing = 0x2000090E;
			hi2c1.Init.OwnAddress1 = 0;
			hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
			hi2c1.Init.OwnAddress2 = 0;
			hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
			hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
			hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
			if (HAL_I2C_Init(&hi2c1) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}

				/**Configure Analogue filter 
				*/
			if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}

				/**Configure Digital filter 
				*/
			if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}
		
  }

  bufferColourSensor[0]= 0x01;

  while(bufferColourSensor[0] !=0)                                                        //Check if the Sensor is done reading
  {
    if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, CTRL, 1, bufferColourSensor, 1, 100) !=HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		if(1==secondFlag) { /* if it is hung here, use this to get out of the while loop */
			/* wait too long, hanging here at reading */
			/* NOTE: this routine does not catch the hang exception! */
			API_USR_TransmitMessage("Waiting too long for Colour Sensor to Read Signals!\r\n");
		}
  }

  HAL_GPIO_WritePin( ColourSensorLed_GPIO_Port, ColourSensorLed_Pin, GPIO_PIN_RESET);                                   //Turn off LED

  address= DATA_RED_LO;
  for (x = 0; x <8; x++)
  {
    bufferColourSensor[0]= 0;
    if(HAL_I2C_Mem_Read(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100) !=HAL_OK)   //Start collecting DATA
		{
			_Error_Handler(__FILE__, __LINE__);
		}

    Data[x]= bufferColourSensor[0];
    address++;

  }
	
	CalculateColourSensorData();	
	
	HAL_GPIO_WritePin( ColourSensorLed_GPIO_Port, ColourSensorLed_Pin, GPIO_PIN_RESET);                                //Turn off LED
  //return DATA[];
}

void CalculateColourSensorData(void)
{
	Red= Data[1]*256+ Data[0];
	Green= Data[3]*256+ Data[2];
	Blue= Data[5]*256+ Data[4];
	Clear= Data[7]*256+ Data[6];

}

void ArrangeColourSensorArrayValues(void)
{
	uint16_t unknown=0;

		for(int j=0;j<NUM_OF_DATA_COLOURSENSOR;j++)
		{
			for(int i=0;i<(NUM_OF_DATA_COLOURSENSOR-1);i++)
			{
				if(RedColourSensorArray[i]>RedColourSensorArray[i+1])
				{
					unknown=RedColourSensorArray[i];
					RedColourSensorArray[i]=RedColourSensorArray[i+1];
					RedColourSensorArray[i+1]=unknown;
				}

				if(GreenColourSensorArray[i]>GreenColourSensorArray[i+1])
				{
					unknown=GreenColourSensorArray[i];
					GreenColourSensorArray[i]=GreenColourSensorArray[i+1];
					GreenColourSensorArray[i+1]=unknown;
				}

				if(BlueColourSensorArray[i]>BlueColourSensorArray[i+1])
				{
					unknown=BlueColourSensorArray[i];
					BlueColourSensorArray[i]=BlueColourSensorArray[i+1];
					BlueColourSensorArray[i+1]=unknown;
				}

				if(ClearColourSensorArray[i]>ClearColourSensorArray[i+1])
				{
					unknown=ClearColourSensorArray[i];
					ClearColourSensorArray[i]=ClearColourSensorArray[i+1];
					ClearColourSensorArray[i+1]=unknown;
				}

			}
		}
}

void PrereadColourSensorData(void)
{
//	uint8_t placing=(NUM_OF_DATA_COLOURSENSOR/2);

	//Set Gains
	ReadColourSensorSetting(); //To see whether preliminary Gain values have been stored
	//API_USR_TransmitMessage("In PrereadColourSensorData, after ReadColourSensorSetting!\r\n");
	if (0 == ClearGain) //Upon Power Reset
	{
		API_USR_TransmitMessage("ClearGain==0\r\n");
		//Implementation #1: Set default values and do auto calibration
		SetSensorGains(); //Set with default values
		CapacitorValue = CAPACITOR;
		SelectCapacitorAmount(CapacitorValue);
		AutoCalibrateSensorGains(); 
		
		//Implementation #2: Read from EEPROM or FLASH or file
		//ReadSensorGainsFromEEPROM();
		//ReadSensorGainsFromFLASH();
		
		//Trim Offset Values
		//TrimOffset();
	}

	//Set Data
/*
	for(int x=0;x<5;x++)
	{
		CollectColourSensorData();
		RedColourSensorArray[x]=Red;
		GreenColourSensorArray[x]=Green;
		BlueColourSensorArray[x]=Blue;
		ClearColourSensorArray[x]=Clear;
	}

	ArrangeColourSensorArrayValues();

	Red=RedColourSensorArray[placing];
	Green=GreenColourSensorArray[placing];
	Blue=BlueColourSensorArray[placing];
	Clear=ClearColourSensorArray[placing];

*/


}

void ColourSensorMenu(void)
{
	
	API_USR_TransmitMessage("/**********************Colour Sensor Menu**********************/\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("* 1- Start Colour Detection Test                              *\r\n");
	API_USR_TransmitMessage("* 2- Set Capacitor Amount                                     *\r\n");
	API_USR_TransmitMessage("* 3- Set Red Channel Gain                                     *\r\n");
	API_USR_TransmitMessage("* 4- Set Green Channel Gain                                   *\r\n");
	API_USR_TransmitMessage("* 5- Set Blue Channel Gain                                    *\r\n");
	API_USR_TransmitMessage("* 6- Set Cleard Channel Gain                                  *\r\n");
	API_USR_TransmitMessage("* 7- Set Default Channel Gains                                *\r\n");
	API_USR_TransmitMessage("* 8- Auto Calibrate Sensor Gains                              *\r\n");
	API_USR_TransmitMessage("* 9- Return To Top Menu                                       *\r\n");
	API_USR_TransmitMessage("* 10- Exit                                                    *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*      C:%02d    RG:%04d    GG:%04d    BG:%04d    CG:%04d       *\r\n",CapacitorValue, RedGain, GreenGain,BlueGain, ClearGain);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*     Red:%04d    Green:%04d    Blue:%04d     Clear:%04d      *\r\n", Red, Green, Blue, Clear);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("*                                                             *\r\n");
	API_USR_TransmitMessage("*                                                             *\r\n");
	snprintf((char *)buffer,sizeof(buffer),"*               Time: %02d:%02d           Date: %02d/%02d             *\r\n",sTime.Hours, sTime.Minutes, sDate.Date, sDate.Month);
	API_USR_TransmitMessage((char *)buffer);
	API_USR_TransmitMessage("/**************************************************************/\r\n\n");
	API_USR_TransmitMessage("Please Select...(Press enter after selection)\r\n\n");
}

void StartColourDetectionTest(void)
{
	ReadColourSensorSetting();
	SetSensorGains();
	SelectCapacitorAmount(CapacitorValue);

	//TrimOffset();
	secondCount = 0;				// reset second counter

	while(1)
	{
		if(HAL_UART_Receive(&huart2,buffer,1,100)==HAL_OK)
		{
			if(ESC == buffer[0])
				SelectColourSensorMenuOnUart();
		}
		else
		{
//			_Error_Handler(__FILE__, __LINE__);
		}

		if(1 == secondFlag)
		{
			secondFlag=0;    /* Move this statement up and check secondFlag==0 in Collect ColourSensorData() to see if it hangs there */
			CollectColourSensorData();

//			snprintf((char *)buffer, sizeof(buffer),"%02d/%02d, %02d:%02d, Red:%04d, Green: %04d, Blue:%04d, Clear:%04d\r\n",sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, Red, Green, Blue, Clear);
//			snprintf((char *)buffer, sizeof(buffer),"%02d/%02d, %02d:%02d:%02d, Red:%04d, Green: %04d, Blue:%04d, Clear:%04d\r\n",sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, secondCount, Red, Green, Blue, Clear);
			snprintf((char *)buffer, sizeof(buffer),"%02d/%02d, %02d:%02d:%02d, R, %04d, G, %04d, B, %04d, C, %04d, R/C, %1.4f, G/C, %1.4f, B/C, %1.4f\r\n",sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, secondCount, Red, Green, Blue, Clear, (float)Red/(float)Clear, (float)Green/(float)Clear, (float)Blue/(float)Clear);
			if(HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100) !=HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
			//secondFlag=0;
		}
	}
}

void SetCapacitorAmount(void)
{
	autoLogin=0;
	//SelectCapacitorAmount(CapacitorValue);

	API_USR_TransmitMessage("\r\nPlease enter the new Capacitor value from 0 to 15.\r\n\r\n");

	CapacitorValue = ReceiveUartCalculationValue();

	//if(CapacitorValue<0 || CapacitorValue>15)
	if(CapacitorValue>15)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is invaild...\r\n\r\n");
		SetCapacitorAmount();
	}

	SelectCapacitorAmount(CapacitorValue);

	SelectColourSensorMenuOnUart();
}

void SetRedGain(void)
{
	API_USR_TransmitMessage("Please enter the Red gain:(0-4095)\r\n\r\n");
	RedGain= ReceiveUartCalculationValue();

	//if(RedGain<0 || RedGain>4095)
	if(RedGain>4095)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is out of range...\r\n\r\n");
		SetRedGain();
	}

	address = INT_RED_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = RedGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_RED_HI;
	bufferColourSensor[0] = (RedGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

   SelectColourSensorMenuOnUart();

}

void SetGreenGain(void)
{
	API_USR_TransmitMessage("Please enter the Green gain:(0-4095)\r\n\r\n");
	GreenGain= ReceiveUartCalculationValue();

	//if(GreenGain<0 || GreenGain>4095)
	if(GreenGain>4095)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is out of range...\r\n\r\n");
		SetGreenGain();
	}

	address= INT_GREEN_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = GreenGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_GREEN_HI;
	bufferColourSensor[0] = (GreenGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	SelectColourSensorMenuOnUart();
}

void SetBlueGain(void)
{
	API_USR_TransmitMessage("Please enter the Blue gain:(0-4095)\r\n\r\n");
	BlueGain= ReceiveUartCalculationValue();

	//if(BlueGain<0 || BlueGain>4095)
	if(BlueGain>4095)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is out of range...\r\n\r\n");
		SetBlueGain();
	}

	address= INT_BLUE_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = BlueGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_BLUE_HI;
	bufferColourSensor[0] = (BlueGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	SelectColourSensorMenuOnUart();
}

void SetClearGain(void)
{
	API_USR_TransmitMessage("Please enter the Clear gain:(0-4095)\r\n\r\n");
	ClearGain= ReceiveUartCalculationValue();

	//if(ClearGain<0 || ClearGain>4095)
	if(ClearGain>4095)
	{
		API_USR_TransmitMessage("\r\n\r\nSorry your input is out of range...\r\n\r\n");
		SetClearGain();
	}

	address= INT_CLEAR_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = ClearGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_CLEAR_HI;
	bufferColourSensor[0] = (ClearGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	SelectColourSensorMenuOnUart();
}

void SetDefaultGain(void)
{
	CapacitorValue = CAPACITOR;
	RedGain = GAIN_RED;
	GreenGain = GAIN_GREEN;
	BlueGain = GAIN_BLUE;
	ClearGain = GAIN_CLEAR;

	//Set default Capacitor Amount
	SelectCapacitorAmount(CapacitorValue);
	
	//Set default Red Channel Gain
	address = INT_RED_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = RedGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_RED_HI;
	bufferColourSensor[0] = (RedGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Red Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}
		
	//Set default Green Channel Gain
	address= INT_GREEN_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = GreenGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_GREEN_HI;
	bufferColourSensor[0] = (GreenGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Green Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	
	//Set default Blue Channel Gain
	address= INT_BLUE_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0] = BlueGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_BLUE_HI;
	bufferColourSensor[0] = (BlueGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Blue Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	
	//Set default Clear Channel Gain
	address= INT_CLEAR_LO;                                                            //Select value range (Sensor gain)
	bufferColourSensor[0]= ClearGain & 0x00FF;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is set(LOW)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	address= INT_CLEAR_HI;
	bufferColourSensor[0]= (ClearGain>>8)& 0x000F;

	if(HAL_I2C_Mem_Write(&hi2c1, COLOUR_SENSOR<<1, address, 1, bufferColourSensor, 1, 100)==HAL_OK)
				API_USR_TransmitMessage("Clear Gain of the Colour Sensor is set(HIGH)!\r\n");
	else
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Back to show menu
	SelectColourSensorMenuOnUart();
}

void AutoCalibrateSensorGains(void)
{
	//Start with the default gains obtained earlier
	//calibrate to get data outputs closest to CALIBRATE_RED, CALIBRATE_GREEN, CALIBRATE_BLUE, CALIBRATE_CLEAR, which are set
	//to be close to the new yellowish NH3 sensor pad
	float multiplier;
	CollectColourSensorData();
	
	snprintf((char *)buffer,sizeof(buffer),"Before calibration:\r\nC:%02d    RG:%04d    GG:%04d    BG:%04d    CG:%04d\r\n",CapacitorValue, RedGain, GreenGain, BlueGain, ClearGain);
	API_USR_TransmitMessage((char *)buffer);
	snprintf((char *)buffer,sizeof(buffer),"        R: %04d    G: %04d    B: %04d     C: %04d\r\n\r\n", Red, Green, Blue, Clear);
	API_USR_TransmitMessage((char *)buffer);
		
	for(int x=1;x<=CALIBRATION_ITERATIONS;x++)
	{		
		multiplier = (21.0 - x)/10.0;
		if(CALIBRATE_RED > Red)	RedGain += (int)((CALIBRATE_RED - Red)*multiplier);
			else if (CALIBRATE_RED < Red) RedGain -= (int)((Red - CALIBRATE_RED)*multiplier);
		if(CALIBRATE_GREEN > Green)	GreenGain += (int)((CALIBRATE_GREEN - Green)*multiplier);
			else if (CALIBRATE_GREEN < Green) GreenGain -= (int)((Green - CALIBRATE_GREEN)*multiplier);
		if(CALIBRATE_BLUE > Blue)	BlueGain += (int)((CALIBRATE_BLUE - Blue)*multiplier);
			else if (CALIBRATE_BLUE < Blue) BlueGain -= (int)((Blue - CALIBRATE_BLUE)*multiplier);
		if(CALIBRATE_CLEAR > Clear)	ClearGain += (int)((CALIBRATE_CLEAR - Clear)*multiplier);
			else if (CALIBRATE_CLEAR < Clear) ClearGain -= (int)((Clear - CALIBRATE_CLEAR)*multiplier);
		
		SetSensorGains();
		CollectColourSensorData();
		
		snprintf((char *)buffer,sizeof(buffer),"After Interation #:%2d\r\nC:%02d    RG:%04d    GG:%04d    BG:%04d    CG:%04d\r\n",x, CapacitorValue, RedGain, GreenGain,BlueGain, ClearGain);
		API_USR_TransmitMessage((char *)buffer);
		snprintf((char *)buffer,sizeof(buffer),"        R: %04d    G: %04d    B: %04d     C: %04d\r\n\r\n", Red, Green, Blue, Clear);
		API_USR_TransmitMessage((char *)buffer);
	}
	
	//HAL_Delay(5000);  //***** Pause to allow triggering Analyzer *****
	TrimOffset();
	
	SelectColourSensorMenuOnUart();
}

void SelectColourSensorMenuOnUart(void)
{
	if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		API_USR_TransmitMessage("Colour Sensor(I2C1) is not ready\r\n\r\n");
		SelectTopMenuOnUart();
	}

	PrereadColourSensorData();
	ColourSensorMenu();
	exitFlag=0;
	autoLogin=0;
/*
	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement];
		switch(menuSelection)
		{
			case ('1'):
				StartColourDetectionTest();
				break;
			case ('2'):
				SetCapacitorAmount();
				break;
			case ('3'):
				SetRedGain();
				break;
			case ('4'):
				SetGreenGain();
				break;
			case ('5'):
				SetBlueGain();
				break;
			case ('6'):
				SetClearGain();
				break;
			case ('7'):
				AutoCalibrateSensorGains();
				break;
			case ('8'):
				SelectTopMenuOnUart();
				break;
			case ('9'):
				SetExit();
				break;
			default:
				autoLogin=0;
				if(menuSelection!='1' || menuSelection!='2' || menuSelection!='3' || menuSelection!='4' || menuSelection!='5'|| menuSelection!='6'|| menuSelection!='7')
				{
					HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
					API_USR_TransmitMessage("\r\nSorry your input is invalid. Please try again...\r\n\r\n\r\n\r\n");
				}
				SelectColourSensorMenuOnUart();
				break;
		}
	}
*/

	while(0 == exitFlag)
	{
		ReceiveUart();
		menuSelection=receiveInputArray[enterPlacement];
//		snprintf((char *)buffer,sizeof(buffer),"enterPlacement: %2d\r\n",enterPlacement);
//		API_USR_TransmitMessage((char *)buffer);
//		snprintf((char *)buffer,sizeof(buffer),"menuSelection: %2d\r\n",menuSelection);
//		API_USR_TransmitMessage((char *)buffer);
		
		menuSelection = menuSelection - 48; // to get integer value from ASCII
		//menuSelection = ReceiveUartCalculationValue();
		//snprintf((char *)buffer,sizeof(buffer),"menuSelection: %2d\r\n",menuSelection);
		//API_USR_TransmitMessage((char *)buffer);
		menuSelection--; //index starts from zero
		
		if(menuSelection >= COUNT_OF_OPERATION(API_CDT_operation)) //check range
		{
			HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
			API_USR_TransmitMessage("\r\nyour input is invalid. Please try again...\r\n\r\n\r\n\r\n");
			SelectTopMenuOnUart();
		}else
		{
			API_CDT_operation[menuSelection].OpFunc();
		}
	}
	
}


