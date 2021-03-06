#include <stdio.h>
#include <stdlib.h>
#include "cy8cmbr3.h"
#include "stripEffects.h"

extern I2C_HandleTypeDef hi2c4;
extern I2C_HandleTypeDef hi2c1;

#define WakeWord_Effect()			setWHOLEcolor(10, 100, 100)

uint8_t TxData[2];
__IO ITStatus isVolumeBtInProcess = RESET;
__IO ITStatus isRecordingBtInProcess = RESET;
extern __IO uint8_t MIC_CHECK;
extern __IO uint8_t BT_EVENTSTATE;
__IO ITStatus isRecordingBtEvent = RESET;
/* Above are the Command Codes used to configure MBR3*/
uint8_t configData[129] = {
	//The below configuration array enables all 4 buttons, Host interrupt
	0x00u, //REMAP_REGISTER
	0x78u, 0x00u, 0x00u, 0x00u, 0x20u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
	0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
	0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
	0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
	0x00u, 0xFFu, 0xFFu, 0xFFu, 0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu,
	0xFFu, 0x00u, 0x00u, 0x00u, 0x04u, 0x03u, 0x01u, 0x58u,
	0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
	0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x65u, 0xDBu
};

// uint8_t configData[129] = {
//     //The below configuration array enables all 4 buttons, Host INT, Proximity
//     0x00u, // REMAP_REGISTER
//     0x78u, 0x00u, 0x00u, 0x00u, 0x20u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
//     0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
//     0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
//     0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
//     0x00u, 0xFFu, 0xFFu, 0xFFu, 0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu,
//     0xFFu, 0x00u, 0x00u, 0x00u, 0x24u, 0x03u, 0x01u, 0x59u,
//     0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x05u, 0x78u
// };

uint8_t *TxBuffer;
uint8_t LED_BTN3_Status(void);

void MBR3_HOST_INT_Config(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;

	/* Enable GPIOD clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure PD1 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin  = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

CY8CMBR3116_Result ConfigureMBR3(void)
{
	uint8_t REGMAP_REG = (uint8_t)REGMAP_ORIGIN;

	//Wake up the MBR3 device
	/* fix me, following cypress document, MBR3 need to be wrote
		data more than 3 times to wake up */ 
	if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
	{
		logs_error("CY3280-MBR3 is not ready");
	}

	// send configuration
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MBR3_ADDR, 
				(uint8_t *)configData, 129, 1000) != HAL_OK)
	{
		logs_error("Send configuration");
		return CY8CMBR3116_Result_ERROR;
	}
	//HAL_Delay(100); //need, if dont run

	//apply configuration to MBR3
	TxBuffer[0] = CTRL_CMD;
	TxBuffer[1] = SAVE_CHECK_CRC;
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MBR3_ADDR, 
				(uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
	{
		logs_error("CRC checking");
		return CY8CMBR3116_Result_ERROR;
	}
	HAL_Delay(100);

	TxBuffer[0] = CTRL_CMD;
	TxBuffer[1] = SW_RESET;
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MBR3_ADDR, 
				(uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
	{
		logs_error("Reset SW");
		return CY8CMBR3116_Result_ERROR;
	}

	//Provide a delay to calculate and save CRC
	HAL_Delay(200);
	return CY8CMBR3116_Result_OK;
}

CY8CMBR3116_Result ReadandDisplaySensorStatus(void)
{
	uint8_t button_stat;
	uint8_t BUTTON_REG = (uint8_t)BUTTON_STATUS;

	/* fix me, following cypress document, MBR3 need to be wrote
		data more than 3 times to wake up */ 
	if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
	{
		logs_error("CY3280-MBR3 is not ready");
	}
	//Config button
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MBR3_ADDR, 
				&BUTTON_REG, 1, 1000) != HAL_OK)
	{
		logs_error("Configure button");
		return CY8CMBR3116_Result_ERROR;
	}

	if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MBR3_ADDR,
				&button_stat, 1, 1000) != HAL_OK)
	{
		logs_error("Read button status");
		return CY8CMBR3116_Result_ERROR;
	}

	DisplaySensorStatus(button_stat);

	return CY8CMBR3116_Result_OK;
}

void DisplaySensorStatus(uint8_t buffer)
{
	static uint8_t touched=0, prox=0;
	uint8_t gpo5_status;

	if(touched)
	{
		touched = 0;
		//logs("Button released");

		/* clear all led */
		LEDx_OnOff(CY8C_LED1_PIN, GPIO_PIN_RESET);
		LEDx_OnOff(CY8C_LED2_PIN, GPIO_PIN_RESET);
		//LEDx_OnOff(CY8C_LED3_PIN, GPIO_PIN_RESET);
		LEDx_OnOff(CY8C_LED4_PIN, GPIO_PIN_RESET);
	}
	else if(buffer & VOLUME_DOWN_BT)
	{
		LEDx_OnOff(CY8C_LED1_PIN, GPIO_PIN_SET);
		//logs("Button 1 TOUCHED");
		touched = 1;

		if(isVolumeBtInProcess == RESET)
		{
			isVolumeBtInProcess = SET;

			/* sending I2C data to Mainboard */
			OUPUT_PIN_GENERATE_PULSE();

			TxData[0] = CYPRESS_BUTTON;
			TxData[1] = VOLUME_DOWN;
			if(HAL_I2C_Slave_Transmit_IT(&hi2c4, (uint8_t*)TxData, 2)!= HAL_OK)
			{
				/* Transfer error in transmission process */
				logs_error("error transfer");
			}
		}
	}
	else if((buffer & VOLUME_UP_BT))
	{
		LEDx_OnOff(CY8C_LED2_PIN, GPIO_PIN_SET);
		//logs("Button 2 TOUCHED");
		touched = 1;
		if(isVolumeBtInProcess == RESET)
		{
			isVolumeBtInProcess = SET;

			/* sending I2C data to Mainboard */
			OUPUT_PIN_GENERATE_PULSE();

			TxData[0] = CYPRESS_BUTTON;
			TxData[1] = VOLUME_UP;
			if(HAL_I2C_Slave_Transmit_IT(&hi2c4, (uint8_t*)TxData, 2)!= HAL_OK)
			{
				/* Transfer error in transmission process */
				logs_error("error transfer");
			}
		}
	}
	else if(buffer & MUTE_MIC_BT)
	{
		//logs("Button 3 TOUCHED");
		touched = 1;
		gpo5_status = LED_BTN3_Status();
		CLEAR_ALL_LEDS();
		if(!gpo5_status)
		{
			LEDx_OnOff(CY8C_LED3_PIN, GPIO_PIN_SET);
			//logs("microphone mute");

			CLEAR_ALL_LEDS();
			setWHOLEcolor(100, 0, 0);
			MIC_CHECK = 1;
		}
		else {
			LEDx_OnOff(CY8C_LED3_PIN, GPIO_PIN_RESET);
			//logs("microphone unmute");
			CLEAR_ALL_LEDS();
			MIC_CHECK = 0;
		}
	}
	else if(buffer & WAKEWORD_BT)
	{
		//logs("Button 4 TOUCHED");
		touched = 1;
		if(isRecordingBtInProcess == RESET)
		{
			isRecordingBtInProcess = SET;

			gpo5_status = LED_BTN3_Status();
			if((gpo5_status) && BT_EVENTSTATE == 0)
			{
				LEDx_OnOff(CY8C_LED4_PIN, GPIO_PIN_SET);
				CLEAR_ALL_LEDS();
				WakeWord_Effect();
				isRecordingBtEvent = SET;
				//logs("record now!!!");
				/* sending I2C data to Mainboard */
				OUPUT_PIN_GENERATE_PULSE();

				TxData[0] = CYPRESS_BUTTON;
				TxData[1] = WAKEWORD_START;
				if(HAL_I2C_Slave_Transmit_IT(&hi2c4, (uint8_t*)TxData, 2) != HAL_OK)
				{
					/* Transfer error in transmission process */
					logs_error("error transfer");
				}
			}
			isRecordingBtInProcess = RESET;
		}
	}
}

uint8_t LED_BTN3_Status(void)
{
	uint8_t gpo_state;
	uint8_t GPO_DATA_REG = (uint8_t)GPO_DATA;

	//GPO_OUTPUT_STATE
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MBR3_ADDR, 
				&GPO_DATA_REG, 1, 1000) != HAL_OK)
	{
		logs_error("GPIO_OUTPUT_STATE");
	}
	if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MBR3_ADDR,
				&gpo_state, 1, 1000) != HAL_OK)
	{
		logs_error("Read button status");
	}

	gpo_state &= GPO_MASK(5);

	return gpo_state;
}