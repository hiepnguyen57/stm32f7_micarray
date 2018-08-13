/**
  ******************************************************************************
  * @file    audio_codec.c
  * @author  Phan Le Son ( porting from "MCD Application Team")
  * @version V1.0.0
  * @date    12-December-2015
  * @brief   This file includes the low layer driver for CS43L22 Audio Codec 
  ******************************************************************************
                                             User NOTES
1. How To use this driver:
--------------------------
   - Call the function AUDIO_Init(
                                    OutputDevice: physical output mode (OUTPUT_DEVICE_SPEAKER, 
                                                 OUTPUT_DEVICE_HEADPHONE, OUTPUT_DEVICE_AUTO or 
                                                 OUTPUT_DEVICE_BOTH)
                                    Volume: initial volume to be set (0 is min (mute), 100 is max (100%)
                                    AudioFreq: Audio frequency in Hz (8000, 16000, 22500, 32000 ...)
                                    this parameter is relative to the audio file/stream type.
                                   )
      This function configures all the hardware required for the audio application (codec, I2C, I2S, 
      GPIOs, DMA and interrupt if needed). This function returns 0 if configuration is OK.
      if the returned value is different from 0 or the function is stuck then the communication with
      the codec (try to un-plug the power or reset device in this case).
      + OUTPUT_DEVICE_SPEAKER: only speaker will be set as output for the audio stream.
      + OUTPUT_DEVICE_HEADPHONE: only headphones will be set as output for the audio stream.
      + OUTPUT_DEVICE_AUTO: Selection of output device is made through external switch (implemented 
         into the audio jack on the board). When the Headphone is connected it is used
         as output. When the headphone is disconnected from the audio jack, the output is
         automatically switched to Speaker.
      + OUTPUT_DEVICE_BOTH: both Speaker and Headphone are used as outputs for the audio stream
         at the same time.
   - Call the function AUDIO_Play(
                                  pBuffer: pointer to the audio data file address
                                  Size: size of the buffer to be sent in Bytes
                                 )
      to start playing (for the first time) from the audio file/stream.
   - Call the function AUDIO_PauseResume(
                                         Cmd: AUDIO_PAUSE (or 0) to pause playing or AUDIO_RESUME (or 
                                               any value different from 0) to resume playing.
                                         )
       Note. After calling AUDIO_PauseResume() function for pause, only AUDIO_PauseResume() should be called
          for resume (it is not allowed to call AUDIO_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   - For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named AUDIO_XXX_CallBack() and only their prototypes are declared in 
      the stm32f4_discovery_audio_codec.h file. (refer to the example for more details on the callbacks implementations)
   - To Stop playing, to modify the volume level or to mute, use the functions
       AUDIO_Stop(), AUDIO_VolumeCtl() and AUDIO_Mute().
 
 Driver architecture:
 --------------------
 This driver is composed of three main layers:
   o High Audio Layer: consists of the function API exported in the audio_codec.h file
     (AUDIO_Init(), AUDIO_Play() ...)
   o Codec Control layer: consists of the functions API controlling the audio codec (CS43L22) and 
     included as local functions in file stm32f4_discovery_audio_codec.c (Codec_Init(), Codec_Play() ...)
   o Media Access Layer (MAL): which consists of functions allowing to access the media containing/
     providing the audio file/stream. These functions are also included as local functions into
     the stm32f4_discovery_audio_codec.c file (Audio_MAL_Init(), Audio_MAL_Play() ...)
  Each set of functions (layer) may be implemented independently of the others and customized when 
  needed.    
2. Modes description:
---------------------
     + AUDIO_MAL_MODE_NORMAL : is suitable when the audio file is in a memory location.
     + AUDIO_MAL_MODE_CIRCULAR: is suitable when the audio data are read either from a 
        memory location or from a device at real time (double buffer could be used).
3. DMA interrupts description:
------------------------------
     + AUDIO_IT_TC_ENABLE: Enable this define to use the DMA end of transfer interrupt.
        then, a callback should be implemented by user to perform specific actions
        when the DMA has finished the transfer.
     + AUDIO_IT_HT_ENABLE: Enable this define to use the DMA end of half transfer interrupt.
        then, a callback should be implemented by user to perform specific actions
        when the DMA has reached the half of the buffer transfer (generally, it is useful 
        to load the first half of buffer while DMA is loading from the second half).
     + AUDIO_IT_ER_ENABLE: Enable this define to manage the cases of error on DMA transfer.
4. Known Limitations:
---------------------
   1- When using the Speaker, if the audio file quality is not high enough, the speaker output
      may produce high and uncomfortable noise level. To avoid this issue, to use speaker
      output properly, try to increase audio file sampling rate (typically higher than 48KHz).
      This operation will lead to larger file size.
   2- Communication with the audio codec (through I2C) may be corrupted if it is interrupted by some
      user interrupt routines (in this case, interrupts could be disabled just before the start of 
      communication then re-enabled when it is over). Note that this communication is only done at
      the configuration phase (AUDIO_Init() or AUDIO_Stop()) and when Volume control modification is 
      performed (AUDIO_VolumeCtl() or AUDIO_Mute()). When the audio data is played, no communication is 
      required with the audio codec.
  3- Parsing of audio file is not implemented (in order to determine audio file properties: Mono/Stereo, Data size, 
     File size, Audio Frequency, Audio Data header size ...). The configuration is fixed for the given audio file.
  4- Mono audio streaming is not supported (in order to play mono audio streams, each data should be sent twice 
     on the I2S or should be duplicated on the source buffer. Or convert the stream in stereo before playing).
  5- Supports only 16-bit audio data size.
*/


/* Includes ------------------------------------------------------------------*/
#include "audio_codec.h"
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal_i2s.h"
#include "audio.h"
#include "main.h"



/** 
  *      This file includes the low layer driver for CS43L22 Audio Codec
  */ 



/* This is an audio file stored in the Flash memory as a constant table of 16-bit data.
    The audio format should be WAV (raw / PCM) 16-bits, Stereo (sampling rate may be modified) */
extern I2S_HandleTypeDef     hi2s3;


/* This structure is declared global because it is handled by two different functions */
DMA_InitTypeDef DMA_InitStructure; 
DMA_InitTypeDef AUDIO_MAL_DMA_InitStructure;
DMA_HandleTypeDef     DmaHandle;
I2C_HandleTypeDef     hi2c1,hi2c2;


__IO uint32_t  CODECTimeout = CODEC_LONG_TIMEOUT;   
__IO uint32_t CurrAudioInterface = AUDIO_INTERFACE_I2S; //AUDIO_INTERFACE_DAC
__IO uint8_t OutputDev = 0;
uint16_t *CurrentPos ;             /* This variable holds the current position of audio pointer */



/* Private typedef -----------------------------------------------------------*/
#define AUDIO_SIZE_ELEMENT (2*AUDIO_OUT_BUFFER_SIZE+10)
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


int16_t PCM_Buffer1[PAR_CHNNL*PAR_N];
int16_t PCM_Buffer2[PAR_CHNNL*PAR_N];


__IO uint16_t cntFrm;
__IO uint8_t  swtBufUSBOut;

extern uint8_t buffer_switch;
extern Mic_Array_Data Buffer1,Buffer2,Buffer3;




extern uint8_t swtCase1Mic56;





/**
  * @brief Initializes IOs used by the Audio Codec (on the control and audio 
  *        interfaces).
  * @param  None
  * @retval None
  */
void Codec_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  
    /*-----------------------*/
    /* ---------PA4: LCCKO(I2S2)-------------*/
    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_4;
    GPIO_InitStructure.Mode =GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull =GPIO_PULLUP;
    GPIO_InitStructure.Speed =GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    //HAL_NVIC_SetPriority((IRQn_Type)EXTI4_IRQn, INTERRUPT_PRI_EXT_LRCK, 0);
    //HAL_NVIC_EnableIRQ((IRQn_Type)EXTI4_IRQn);

     /* ---------PB12: LCCKO (I2S2)-------------*/
    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_12;
    GPIO_InitStructure.Mode =GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull =GPIO_PULLUP;
    GPIO_InitStructure.Speed =GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
    /*-----------------------------------------*/

    /* ---------PA15: LCCKO --------------------*/
    __GPIOE_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    GPIO_InitStructure.Mode =GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull =GPIO_PULLUP;
    GPIO_InitStructure.Speed =GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
    /*-----------------------------------------*/

    /*---------PE13: POWER DOWN-----------------*/
    __GPIOE_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_13;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(500);
    /*----------------------------------------*/
  
}



/*========================
                Audio MAL Interface Control Functions
                                                ==============================*/

void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
   GPIO_InitTypeDef GPIO_InitStructure;
	
   
   GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2s->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
  /* Peripheral clock enable */
  __SPI1_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /**I2S1 GPIO Configuration    
    PA4     ------> I2S1_WS --> LRCKO
    PA5     ------> I2S1_CK --> BICKO
    PA7     ------> I2S1_SD --> SDO12
    */
	GPIO_InitStruct.Pin =  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(SPI1_IRQn, INTERRUPT_PRI_SDO12, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(hi2s->Instance==SPI2)
  {
	  /* USER CODE BEGIN SPI2_MspInit 0 */
	  
	  /* USER CODE END SPI2_MspInit 0 */
		/* Peripheral clock enable */
		__SPI2_CLK_ENABLE();
		__GPIOI_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
	  
		/**I2S2 GPIO Configuration	   
		PC1 	------> I2S2_SD  : PI3 PC1 PC3 PB15 	
		PB13	 ------> I2S2_CK :PD3 PB10 PB13 PA9 PI1 
		PB12	 ------> I2S2_WS : PB12 PI0 PB4 PB9  
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_1; //SD
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  
		GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;//WS --> GPIO_PIN_12
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  
	  
		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(SPI2_IRQn, INTERRUPT_PRI_SDO34, 1);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);
		/* USER CODE BEGIN SPI2_MspInit 1 */
	  
	  /* USER CODE END SPI2_MspInit 1 */

  }
  else if(hi2s->Instance==SPI3)
  {


  
    /**I2S3 GPIO Configuration    
       PB2     ------> I2S3_SD --> PC12
       PA15     ------> I2S3_WS (LRCK)
       PB3     ------> I2S3_CK --> PC10
	   PC7    ------> MCLK
    */
 
  /* USER CODE BEGIN SPI3_MspInit 1 */
  __SPI3_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  GPIO_InitStructure.Pin = GPIO_PIN_12; 
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_10; 
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(SPI3_IRQn, INTERRUPT_PRI_SDO56 , 0);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);


 }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI3)
  {   
    /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    HAL_GPIO_DeInit(CODEC_I2S_GPIO, CODEC_I2S_SCK_PIN);
    HAL_GPIO_DeInit(CODEC_I2S_GPIO, CODEC_I2S_SD_PIN);
    HAL_GPIO_DeInit(CODEC_I2S_WS_GPIO, CODEC_I2S_WS_PIN);
	HAL_GPIO_DeInit(CODEC_I2S_MCK_GPIO, CODEC_I2S_MCK_PIN);

    /*##-3- Disable the DMA ####################################################*/
    /* De-Initialize the DMA associated to transmission process */
    HAL_DMA_DeInit(&DmaHandle);


    /*##-4- Disable the NVIC for DMA ###########################################*/
    HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);
  }
}


// void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
// {
//     /*##-1- Reset peripherals ##################################################*/
//     __HAL_RCC_I2C1_FORCE_RESET();
//     __HAL_RCC_I2C1_RELEASE_RESET();


//   ##-2- Disable peripherals and GPIO Clocks #################################
//   /* Configure I2C Tx as alternate function  */
//   HAL_GPIO_DeInit(CODEC_I2C_GPIO, CODEC_I2C_SCL_PIN);
//   /* Configure I2C Rx as alternate function  */
//   HAL_GPIO_DeInit(CODEC_I2C_GPIO, CODEC_I2C_SDA_PIN);
// }


void AudioUSBSend(uint16_t idxFrm) /* This function called every ms */
{
    //Send_Audio_to_USB((int16_t *)PCM_Buffer1, AUDIO_OUT_BUFFER_SIZE*AUDIO_CHANNELS);
    
    (swtBufUSBOut)?Send_Audio_to_USB((int16_t *)&PCM_Buffer2[(PAR_CHNNL*AUDIO_SAMPLING_FREQUENCY/1000)*idxFrm], (PAR_CHNNL*AUDIO_SAMPLING_FREQUENCY/1000)):
                   Send_Audio_to_USB((int16_t *)&PCM_Buffer1[(PAR_CHNNL*AUDIO_SAMPLING_FREQUENCY/1000)*idxFrm], (PAR_CHNNL*AUDIO_SAMPLING_FREQUENCY/1000));
    if (idxFrm == (PAR_N/(AUDIO_SAMPLING_FREQUENCY/1000) -1) ) swtBufUSBOut^=0x01;				   		   
}

/* This function should be called after data processing */
/*-------------------------------------------------------------------------------------------------------------
			  
	Sequence  Record Data                     Processing Data                 Player Data
			  
	1-------  Buffer1                         Buffer2                         Buffer3
			  
	2-------  Buffer3                         Buffer1                         Buffer2		  
			  
	3-------  Buffer2                         Buffer3                         Buffer1 
 ---------------------------------------------------------------------------------------------------------------*/

void AudioPlayerUpd(void) /* This function called with period of 64ms */
{
	switch (buffer_switch)
    {
      case BUF1_PLAY:
		for (uint16_t i=0;i<PAR_N;i++)
		{
            if (swtBufUSBOut)
            {               
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer1[PAR_CHNNL*(i) +j]= *(&Buffer1.bufMIC1[i] + PAR_N*(j));
                }
            }
		    else
		    {
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer2[PAR_CHNNL*(i) +j]= *(&Buffer1.bufMIC1[i] + PAR_N*(j));
                }
		    }
	  
		}
        break;    
      case BUF2_PLAY:
          for (uint16_t i=0;i<PAR_N;i++)
          {
            if (swtBufUSBOut)
            {               
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer1[PAR_CHNNL*(i) +j]= *(&Buffer2.bufMIC1[i] + PAR_N*(j));
                }
            }
		    else
		    {
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer2[PAR_CHNNL*(i) +j]= *(&Buffer2.bufMIC1[i] + PAR_N*(j));
                }
		    }             
          
            
          }
          break;
      case BUF3_PLAY:
		for (uint16_t i=0;i<PAR_N;i++)
		{
            if (swtBufUSBOut)
            {               
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer1[PAR_CHNNL*(i) +j]= *(&Buffer3.bufMIC1[i] + PAR_N*(j));
                }
            }
		    else
		    {
                for (uint16_t j = 0; j < PAR_CHNNL; j++)
                {
                   PCM_Buffer2[PAR_CHNNL*(i) +j]= *(&Buffer3.bufMIC1[i] + PAR_N*(j));
                }
		    }
		} 	
        break;
      default:
        break;
    }
}


