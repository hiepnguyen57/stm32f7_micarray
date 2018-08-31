
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
#include "cy8cmbr3.h"
#include "stripEffects.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "waverecorder.h"
#include "DOA.h"
#include "pdm_filter.h"
#include "DSP.h"
#include "arm_math.h"
/** @addtogroup STM32F7xx_HAL_Examples
	* @{
	*/

/** @addtogroup UART_TwoBoards_ComIT
	* @{
	*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2CX_TIMING             0x40912732 //0x40912732 //0x00303D5D; 0x00A0689A
#define I2C_ADDRESS 			0xD0
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[] = "Hello Worldd";
uint8_t aRxBuffer[RXBUFFERSIZE];
CY8CMBR3116_Result result;
uint8_t STATE = 0;
uint8_t MIC_CHECK = 0;
LPTIM_HandleTypeDef             LptimHandle;

extern Mic_Array_Data Buffer1,Buffer2,Buffer3;

extern uint16_t WaveRecord_flgIni;
extern uint32_t EnergySound,EnergyError;

extern __IO int16_t SPI1_stNipple,I2S1_stNipple, I2S2_stNipple,SPI4_stNipple;
extern __IO   uint8_t I2S1_stPosShft,I2S2_stPosShft,SPI4_stPosShft;
extern USBD_AUDIO_ItfTypeDef  USBD_AUDIO_fops;
extern __IO uint8_t  swtBufUSBOut;
extern __IO uint8_t flgRacing;
extern __IO GPIO_PinState stMIC56;
extern __IO GPIO_PinState stMIC56Old;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

USBH_HandleTypeDef hUSBHost;
USBD_HandleTypeDef hUSBDDevice;
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;//APPLICATION_IDLE

SPI_HandleTypeDef hspi4;
GPIO_InitTypeDef GPIO_INS;
Mic_Array_Coef_f FacMic;
Audio_Out OutData;	  


int16_t bufBeam1[ PAR_N  ];
int16_t bufBeam2[ PAR_N  ];

float CrssCorVal78,CrssCorVal14,CrssCorVal25,CrssCorVal63;

__IO uint16_t  WaveRec_idxSens4,WaveRec_idxSens3;
__IO uint16_t  WaveRec_idxSens1,WaveRec_idxSens2;
__IO uint16_t  WaveRec_idxSens5,WaveRec_idxSens6;
__IO uint8_t   flgDlyUpd; 
__IO uint8_t   flg10ms;

uint8_t buffer_switch = BUF3_PLAY; /* record data to buffer1 */
uint8_t Command_index=1;
uint8_t swtCase1Mic56;
uint8_t Direction;

#if (DEBUG)
uint8_t  pUARTBuf[128];
#endif

extern __IO uint16_t idxFrmUSB;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void USB_Audio_Config(void);
void PWMInit(void);
void I2C1_Init(void);
void I2C4_Init(void);
void GPIO_INT_Init(void);
void Control_Handler(void);
/* Private functions ---------------------------------------------------------*/
void OUPUT_PIN_GENERATE_PULSE(void)
{
	HAL_GPIO_WritePin(CPU_OUTPUT_GPIO_PORT, CPU_OUTPUT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CPU_OUTPUT_GPIO_PORT, CPU_OUTPUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CPU_OUTPUT_GPIO_PORT, CPU_OUTPUT_PIN, GPIO_PIN_SET);
}
/*--------------INLINE FUNCTION-----------------------------------------------*/
inline static void BF_Update(void)
{       
	  /* Hafl buffer is filled in by I2S data stream in */
	  if((flgDlyUpd==0))
	  {
			uint8_t stDirBeam, stOption;

			stDirBeam = 0;
			stOption =1;

			flgDlyUpd = 1; 
/*-------------------------------------------------------------------------------------------------------------
			  
	Sequence  Record Data                     Processing Data                 Player Data
			  
	1-------  Buffer1                         Buffer2                         Buffer3
			  
	2-------  Buffer3                         Buffer1                         Buffer2		  
			  
	3-------  Buffer2                         Buffer3                         Buffer1 
 ---------------------------------------------------------------------------------------------------------------*/
			/* Processing Data */
			switch (buffer_switch)  //buffer_switch
			{             
				case BUF1_PLAY:                    
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer3);

					// if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					//   {
					//      ////Error_Handler();
					//   }
					
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer3,stDirBeam,(int16_t *)bufBeam1);

						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer3.bufMIC1[i] = bufBeam1[i];
						}
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer3,stDirBeam,OutData);

						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer3.bufMIC1[i] = bufBeam1[i];
							 Buffer3.bufMIC2[i] = bufBeam2[i];
						}
					}               
					break;
				case BUF2_PLAY:
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer1);
					// if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					// {
					//     ////Error_Handler();
					// }
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer1,stDirBeam,(int16_t *)bufBeam1);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer1.bufMIC1[i] = bufBeam1[i];
						}
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer1,stDirBeam,OutData);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer1.bufMIC1[i] = bufBeam1[i];
							 Buffer1.bufMIC2[i] = bufBeam2[i];
						}
					 }
				break;
							
			   case BUF3_PLAY:
				
					/* Sound Source Localization */
					Direction = DOACalc(&Buffer2);
					// if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, Direction*60-1) != HAL_OK)
					// {
					//      ////Error_Handler();
					// }
					if (stOption==0)
					{
						/* Summing in Buffer3 */
						BeamFormingSD(&Buffer2,stDirBeam,(int16_t *)bufBeam1);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer2.bufMIC1[i] = bufBeam1[i];
						}                        
					}
					else
					{
						OutData.bufMIC1 = bufBeam1;
						OutData.bufMIC2 = bufBeam2;
						/* Beam on forward and backward of sound source */
						BeamFormingSDCom(&Buffer2,stDirBeam,OutData);
						for (uint16_t i=0; i< PAR_N; i++)
						{
							 Buffer2.bufMIC1[i] = bufBeam1[i];
							 Buffer2.bufMIC2[i] = bufBeam2[i];
						}
					 }

				break;
							
				default:
					break;
			   
		}
			
		AudioPlayerUpd();
	  }

}


inline static void Audio_Play_Out(void)
{

/*-------------------------------------------------------------------------------------------------------------
			  
	Sequence  Record Data                     Processing Data                 Player Data
			  
	1-------  Buffer1                         Buffer2                          Buffer3
			  
	2-------  Buffer3                         Buffer1                           Buffer2		  
			  
	3-------  Buffer2                         Buffer3                           Buffer1 
 ---------------------------------------------------------------------------------------------------------------*/
	flgRacing=0;

#if USB_STREAMING
	AudioUSBSend(idxFrmUSB);
#endif

	++idxFrmUSB;
	/* if player is finished for curent buffer                                  */ 
	if (idxFrmUSB == PAR_N/(AUDIO_SAMPLING_FREQUENCY/1000))
	{

	   RESET_IDX
	   idxFrmUSB=0;
	  switch (buffer_switch)
	  {
		  case BUF1_PLAY:
			  /* set flag for switch buffer */		  
			  buffer_switch = BUF3_PLAY;
			  break;
		   case BUF2_PLAY:
			  /* set flag for switch buffer */
			  buffer_switch = BUF1_PLAY;        
			  break;
		   case BUF3_PLAY:
			  /* set flag for switch buffer */		  
			  buffer_switch = BUF2_PLAY;
			  break;
		   default:
			  break;
	  }
	}
}

void Button_Event(uint8_t Command)
{
	uint8_t led_num;
	switch(Command)
	{
		case VOLUME_UP:
			CLEAR_ALL_LEDS();
			for(led_num = 0; led_num < aRxBuffer[2]; led_num++)
			{
				setLEDcolor(led_num, 100, 100, 100);
			}
			HAL_Delay(1000);
			CLEAR_ALL_LEDS();
			break;

		case VOLUME_DOWN:
			CLEAR_ALL_LEDS();
			for(led_num = 0; led_num < aRxBuffer[2]; led_num++)
			{
				setLEDcolor(led_num, 100, 100, 100);
			}
			HAL_Delay(1000);
			CLEAR_ALL_LEDS();
			break;

		case VOLUME_MUTE:
			CLEAR_ALL_LEDS();
			setWHOLEcolor(100, 100, 0);
			HAL_Delay(1000);
			CLEAR_ALL_LEDS();
			break;

		case VOLUME_UNMUTE:
			CLEAR_ALL_LEDS();
			for(led_num = 0; led_num < aRxBuffer[2]; led_num++)
			{
				setLEDcolor(led_num, 100, 100, 100);
			}
			HAL_Delay(1000);
			CLEAR_ALL_LEDS();
			break;

		case MICROPHONE_MUTE:
			CLEAR_ALL_LEDS();
			setWHOLEcolor(100, 0, 0);
			MIC_CHECK = 1;
			break;

		case MICROPHONE_UNMUTE:
			CLEAR_ALL_LEDS();
			MIC_CHECK = 0;
			break;
	}
}

void User_Event(uint8_t Command)
{
	switch(Command)
	{
		case WAKE_WORD_STOP:
			stripEffect_AlternateColors(1000, 10, 50, 0, 0, 0, 0, 50);
			CLEAR_ALL_LEDS();
			break;

		case WIFI_DISCONNECTED:
			CLEAR_ALL_LEDS();
			setWHOLEcolor(255, 128, 0);
			break;

		case WIFI_CONNECTED:
			CLEAR_ALL_LEDS();
			setWHOLEcolor(0, 255, 0);
			HAL_Delay(3000);
			CLEAR_ALL_LEDS();
			break;

		case MICROPHONE_MUTE:
			CLEAR_ALL_LEDS();
			setWHOLEcolor(100, 0, 0);
			MIC_CHECK = 1;
			break;

		case MICROPHONE_UNMUTE:
			CLEAR_ALL_LEDS();
			MIC_CHECK = 0;
			break;
	}
}

void Control_Handler(void)
{
	switch(aRxBuffer[0])
	{
		case LED_RING:
			break;
		case MIC_ARRAY:
			break;
		case CYPRESS_BUTTON:
			Button_Event(aRxBuffer[1]);
			break;
		case USER_EVENT:
			User_Event(aRxBuffer[1]);
			break;
		default:
			logs("Nothing");
			break;
	}
}
/**
	* @brief  Main program
	* @param  None
	* @retval None
	*/
int main(void)
{
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();
	HAL_Init();

	/* Configure the system clock to 216 MHz */
	SystemClock_Config();
	log_init();

	BSP_AUDIO_OUT_ClockConfig(AUDIO_FREQ, NULL);

	/* Pin configuration for audio */
	Codec_GPIO_Init();

	/* Configure EXTI Line0 (connected to PB0 pin) in interrupt mode */
	MBR3_HOST_INT_Config();
	GPIO_INT_Init();

	//Config i2c bus
	I2C1_Init();
	I2C4_Init();

	STA321MP_Ini();

	/* Configure MBR3 */
	result = ConfigureMBR3();
	if(result != CY8CMBR3116_Result_OK)
	{
		logs_error("Configure MBR3");
	}

	/* Configure LED RING */
	ws281x_init();
	setWHOLEcolor(10, 0, 0);

	/* PWM output */
	PWMInit();

	/* Init Superdirective Beamforming */
	BeamFormingSD_Init();

	/* 2 channels:16Khz Audio USB */
	USB_Audio_Config();

	__disable_irq();
	MIC1TO6_Init();
	StartRecord();
	__enable_irq();

	while(1)
	{
		if(STATE == 0)
		{
			if(!MIC_CHECK)
			{
				BF_Update();
				if (flg10ms==1)
				{
					flg10ms=0;  
#if DEBUG
					sprintf((char *)(pUARTBuf),"Direction: %3d\r\n",Direction*60);
					printf("%s\r\n", pUARTBuf);
#endif
				}
			}

		}
		else
		{
			Control_Handler();
			STATE = 0;
		}
	}
}


/**
	* @brief  System Clock Configuration
	*         The system Clock is configured as follow : 
	*            System Clock source            = PLL (HSE)
	*            SYSCLK(Hz)                     = 216000000
	*            HCLK(Hz)                       = 216000000
	*            AHB Prescaler                  = 1
	*            APB1 Prescaler                 = 4
	*            APB2 Prescaler                 = 2
	*            HSE Frequency(Hz)              = 25000000
	*            PLL_M                          = 25
	*            PLL_N                          = 432
	*            PLL_P                          = 2
	*            PLL_Q                          = 9
	*            VDD(V)                         = 3.3
	*            Main regulator output voltage  = Scale1 mode
	*            Flash Latency(WS)              = 7
	* @param  None
	* @retval None
	*/

void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;  // 432
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;

	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if(ret != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Activate the OverDrive to reach the 216 MHz Frequency */
	ret = HAL_PWREx_EnableOverDrive();
	if(ret != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48 ;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;

  
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4; 
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;


	ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 
	if(ret != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
  
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	if(ret != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
  
	//sop1hc 344/7 = 49.142 MHz
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = 8;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 344;//244
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 7;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 7;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);  
}

void BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
	/* Configure PLLSAI prescalers */
	/* PLLI2S_VCO: VCO_429M
	SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
	SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 214.5/19 = 11.289 Mhz */
	RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
	RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
	HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
	/* SAI clock config
	PLLI2S_VCO: VCO_344M
	SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
	SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 49.142/1 = 49.142 Mhz */
	RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	//RCC_ExCLKInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
	//RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;//244
	RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
	//RCC_ExCLKInitStruct.PLLI2S.PLLI2SR = 1;
	RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
	HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  
}

/**
  * @brief  This function configure I2C1 bus.
  * @param  None
  * @retval None
  */
void I2C1_Init(void)
{
	/*##Configure the I2C peripheral ######################################*/
	hi2c1.Instance              = I2Cx_MASTER;
	hi2c1.Init.Timing           = I2CX_TIMING;
	hi2c1.Init.OwnAddress1      = 0;
	hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2      = 0;
	hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		/* Initialization Error */
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

/**
  * @brief  This function configure I2C1 bus.
  * @param  None
  * @retval None
  */
void I2C4_Init(void)
{
	GPIO_InitTypeDef  			GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef  	RCC_PeriphCLKInitStruct;

	  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx_CPU;
	RCC_PeriphCLKInitStruct.I2c4ClockSelection = RCC_I2Cx_CPU_CLKSOURCE_SYSCLK;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

	/*##-2- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	I2Cx_CPU_SCL_GPIO_CLK_ENABLE();
	I2Cx_CPU_SDA_GPIO_CLK_ENABLE();

	  /* Enable I2Cx clock */
	I2Cx_CPU_CLK_ENABLE();

	/*##-3- Configure peripheral GPIO ##########################################*/
	  /** I2C1 GPIO configuration
		PD12 ------> I2C4_SCL
		PD13 ------> I2C4_SDA
	*/
	/* I2C SCL GPIO pin configuration  */
	GPIO_InitStruct.Pin       = I2Cx_CPU_SCL_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = I2Cx_CPU_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_CPU_SCL_GPIO_PORT, &GPIO_InitStruct);
	  
	/* I2C SDA GPIO pin configuration  */
	GPIO_InitStruct.Pin       = I2Cx_CPU_SDA_PIN;
	GPIO_InitStruct.Alternate = I2Cx_CPU_SCL_SDA_AF;
	HAL_GPIO_Init(I2Cx_CPU_SDA_GPIO_PORT, &GPIO_InitStruct);
	
	  /* NVIC for I2Cx */
	HAL_NVIC_SetPriority(I2Cx_CPU_ER_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(I2Cx_CPU_ER_IRQn);
	HAL_NVIC_SetPriority(I2Cx_CPU_EV_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(I2Cx_CPU_EV_IRQn);

	/*##Configure the I2C peripheral ######################################*/
	hi2c4.Instance              = I2Cx_CPU;
	hi2c4.Init.Timing           = I2CX_TIMING;
	hi2c4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	hi2c4.Init.OwnAddress1      = I2C_ADDRESS;

	if(HAL_I2C_Init(&hi2c4) != HAL_OK)
	{
		/* Initialization Error */
		_Error_Handler(__FILE__, __LINE__);
	}
	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE);
}

void GPIO_INT_Init(void)
{
    GPIO_InitTypeDef    GPIO_Init;

    CPU_INPUT_GPIO_CLK_ENABLE();
    CPU_OUTPUT_GPIO_CLK_ENABLE();

    //fixed error cut PD0 line
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// Configure PE0 as output pin
    GPIO_Init.Pin   = CPU_OUTPUT_PIN;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_PULLUP;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(CPU_OUTPUT_GPIO_PORT, &GPIO_Init);

    /* Configure PB7 pin as input floating */
    GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Pin  = CPU_INPUT_PIN;
    HAL_GPIO_Init(CPU_INPUT_GPIO_PORT, &GPIO_Init);

    //fixed error cut PD0 line, remove in new version
    GPIO_Init.Mode = GPIO_MODE_INPUT;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Pin  = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOD, &GPIO_Init);

    /* Enable adn set EXTI Line7 Interrupt */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{

}
void SubFrameFinished(void)
{
	Audio_Play_Out();
}

void PWMInit(void)
{
	RCC_PeriphCLKInitTypeDef        RCC_PeriphCLKInitStruct;

	/* ### - 1 - Re-target the LSE to Clock the LPTIM Counter ################# */
	/* Select the LSE clock as LPTIM peripheral clock */
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
	RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;  
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

	/* ### - 2 - Initialize the LPTIM peripheral ############################## */
	/*
	*  Instance        = LPTIM1
	*  Clock Source    = APB or LowPowerOSCillator (in this example LSI is
	*                    already selected from the RCC stage)
	*  Counter source  = External event.
	*  Clock prescaler = 1 (No division)
	*  Counter Trigger = Software trigger
	*  Output Polarity = High
	*  Update mode     = Immediate (Registers are immediately updated after any
	*                    write access) 
	*/

	LptimHandle.Instance = LPTIM1;
	LptimHandle.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	LptimHandle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;  
	LptimHandle.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;
	LptimHandle.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE; 
	LptimHandle.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
	LptimHandle.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;

	/* Initialize LPTIM peripheral according to the passed parameters */
	if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* ### - 3 - Start counting in interrupt mode ############################# */
	/*
	*  Period = 99
	*  Pulse  = 49
	*  According to this configuration, the duty cycle will be equal to 50%
	*/
	if (HAL_LPTIM_PWM_Start(&LptimHandle, 359, 0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void EXTI4_IRQHandler(void)
{
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
	}
}


/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* EXTI line 0 interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		//Read button status
		result = ReadandDisplaySensorStatus();
		if(result != CY8CMBR3116_Result_OK)
		{
			logs_error("CYPRESS read status");
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_0);

}

/**
  * @brief  This function handles External line [9:5] interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	/* EXTI line 7 interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
	{
		//receiving data from Mainboard
		logs("Receive I2C Data from Mainboard");
		if(HAL_I2C_Slave_Receive(&hi2c4, (uint8_t *)aRxBuffer, 3, 10000) == HAL_OK)
		{
				printf("%#x\r\n", aRxBuffer[0]);
				printf("%#x\r\n", aRxBuffer[1]);
				printf("%#x\r\n", aRxBuffer[2]);
		}
		STATE = 1;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_7);

}


static void USB_Audio_Config(void)
{
#if (USB_STREAMING)	
	/* Initialize USB descriptor basing on channels number and sampling frequency */
	USBD_AUDIO_Init_Microphone_Descriptor(&hUSBDDevice, AUDIO_SAMPLING_FREQUENCY, AUDIO_CHANNELS);
	/* Init Device Library */
	USBD_Init(&hUSBDDevice, &AUDIO_Desc, 0);
	/* Add Supported Class */
	USBD_RegisterClass(&hUSBDDevice, &USBD_AUDIO);
	/* Add Interface callbacks for AUDIO Class */  
	USBD_AUDIO_RegisterInterface(&hUSBDDevice, &USBD_AUDIO_fops);
	/* Start Device Process */
	USBD_Start(&hUSBDDevice);                          
#endif 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

/**
	* @brief  CPU L1-Cache enable.
	* @param  None
	* @retval None
	*/
static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT
/**
	* @brief  Reports the name of the source file and the source line number
	*         where the assert_param error has occurred.
	* @param  file: pointer to the source file name
	* @param  line: assert_param error line source number
	* @retval None
	*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
	* @}
	*/

/**
	* @}
	*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
