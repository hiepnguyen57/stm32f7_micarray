

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIOCODEC_H
#define __AUDIOCODEC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_gpio_ex.h"
#include "stm32f7xx_hal_i2c.h"


/*------------------------------------
             CONFIGURATION: Audio Codec Driver Configuration parameters
                                      ----------------------------------------*/
#define AUDIO_FILE_SZE          (990000 -22*25000)
#define AUIDO_START_ADDRESS     58 /* Offset relative to audio file header size */
	  
/* Audio Transfer mode (I2S Interrupt) */
//#define I2S_INTERRUPT                 /* Uncomment this line to enable audio transfert with I2S interrupt*/ 

/* Audio Transfer mode (DMA, Interrupt or Polling) */
#define AUDIO_MAL_MODE_NORMAL         /* Uncomment this line to enable the audio 
                                         Transfer using DMA */
/* #define AUDIO_MAL_MODE_CIRCULAR */ /* Uncomment this line to enable the audio 
                                         Transfer using DMA */

/* For the DMA modes select the interrupt that will be used */
#define AUDIO_MAL_DMA_IT_TC_EN        /* Uncomment this line to enable DMA Transfer Complete interrupt */
/* #define AUDIO_MAL_DMA_IT_HT_EN */  /* Uncomment this line to enable DMA Half Transfer Complete interrupt */
/* #define AUDIO_MAL_DMA_IT_TE_EN */  /* Uncomment this line to enable DMA Transfer Error interrupt */

/* Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define AUDIO_IRQ_PREPRIO           0   /* Select the preemption priority level(0 is the highest) */
#define AUDIO_IRQ_SUBRIO            0   /* Select the sub-priority level (0 is the highest) */

/* Uncomment the following line to use the default Codec_TIMEOUT_UserCallback() 
   function implemented in stm32f4_discovery_audio_codec.c file.
   Codec_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occurs during communication (waiting on an event that doesn't occur, bus 
   errors, busy devices ...). */   
/* #define USE_DEFAULT_TIMEOUT_CALLBACK */

/* Enable this define to use the I2S DMA for writing into DAC register */
//#define DAC_USE_I2S_DMA
/*----------------------------------------------------------------------------*/

/*------------------------------------
                    OPTIONAL Configuration defines parameters
                                      ----------------------------------------*/
/* I2C clock speed configuration (in Hz) 
  WARNING: 
   Make sure that this define is not already declared in other files (ie. 
  stm322xg_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                        100000
#endif /* I2C_SPEED */

/* Uncomment defines below to select standard for audio communication between 
  Codec and I2S peripheral */



/* Uncomment the defines below to select if the Master clock mode should be 
  enabled or not */
//#define CODEC_MCLK_ENABLED
#define CODEC_MCLK_DISABLED

/* Uncomment this line to enable verifying data sent to codec after each write 
  operation */
//#define VERIFY_WRITTENDATA 
/*----------------------------------------------------------------------------*/

/*-----------------------------------
                    Hardware Configuration defines parameters
                                     -----------------------------------------*/
/* Audio Reset Pin definition */

#define AUDIO_RESET_PIN                GPIO_PIN_4    
#define AUDIO_RESET_GPIO               GPIOD 
                 
/* I2S peripheral configuration defines */
#define CODEC_I2S                      SPI3
#define CODEC_I2S_ADDRESS              0x40003C0C
#define CODEC_I2S_IRQ                  SPI3_IRQn
#define CODEC_I2S_WS_PIN               GPIO_PIN_15  //PA15
#define CODEC_I2S_SCK_PIN              GPIO_PIN_3   //PB3
#define CODEC_I2S_SD_PIN               GPIO_PIN_2   //PB2 
#define CODEC_I2S_MCK_PIN              GPIO_PIN_7   //PC7
#define CODEC_I2S_GPIO                 GPIOB
#define CODEC_I2S_WS_GPIO              GPIOA
#define CODEC_I2S_MCK_GPIO             GPIOC
//#define Audio_I2S_IRQHandler           SPI3_IRQHandler
#define I2S_INTERRUPT


 #define AUDIO_MAL_DMA_PERIPH_DATA_SIZE DMA_PDATAALIGN_HALFWORD //DMA_PeripheralDataSize_HalfWord
 #define AUDIO_MAL_DMA_MEM_DATA_SIZE    DMA_MDATAALIGN_HALFWORD //DMA_MemoryDataSize_HalfWord



 #define DAC_DHR12L1_ADDRESS            0x4000740C
 #define DAC_DHR12R1_ADDRESS            0x40007408
 #define DAC_DHR8R1_ADDRESS             0x40007410
 #define AUDIO_DAC_CHANNEL              DAC_CHANNEL_1

 /* I2S DMA Stream definitions */
 #define AUDIO_I2S_DMA_STREAM           DMA1_Stream7
 #define AUDIO_I2S_DMA_DREG             CODEC_I2S_ADDRESS
 #define AUDIO_I2S_DMA_CHANNEL          DMA_CHANNEL_0
 #define AUDIO_I2S_DMA_IRQ              DMA1_Stream7_IRQn
 #define AUDIO_I2S_DMA_FLAG_TC          DMA_FLAG_TCIF3_7
 #define AUDIO_I2S_DMA_FLAG_HT          DMA_FLAG_HTIF3_7
 #define AUDIO_I2S_DMA_FLAG_FE          DMA_FLAG_FEIF3_7
 #define AUDIO_I2S_DMA_FLAG_TE          DMA_FLAG_TEIF3_7
 #define AUDIO_I2S_DMA_FLAG_DME         DMA_FLAG_DMEIF3_7

 #define Audio_MAL_I2S_IRQHandler       DMA1_Stream7_IRQHandler


 /* DAC DMA Stream definitions */
 #define AUDIO_DAC_DMA_CLOCK            RCC_AHB1Periph_DMA1
 #define AUDIO_DAC_DMA_STREAM           DMA1_Stream0
 #define AUDIO_DAC_DMA_DREG             DAC_DHR12L1_ADDRESS
 #define AUDIO_DAC_DMA_CHANNEL          DMA_CHANNEL_0
 #define AUDIO_DAC_DMA_IRQ              DMA1_Stream0_IRQn
 #define AUDIO_DAC_DMA_FLAG_TC          DMA_FLAG_TCIF0_4
 #define AUDIO_DAC_DMA_FLAG_HT          DMA_FLAG_HTIF0_4
 #define AUDIO_DAC_DMA_FLAG_FE          DMA_FLAG_FEIF0_4
 #define AUDIO_DAC_DMA_FLAG_TE          DMA_FLAG_TEIF0_4
 #define AUDIO_DAC_DMA_FLAG_DME         DMA_FLAG_DMEIF0_4

 #define Audio_MAL_DAC_IRQHandler       DMA1_Stream0_IRQHandler


/* I2C peripheral configuration defines (control interface of the audio codec) */
#define CODEC_I2C                      I2C1
#define CODEC_I2C_CLK                  RCC_APB1Periph_I2C1
#define CODEC_I2C_GPIO_CLOCK           RCC_AHB1Periph_GPIOB
#define CODEC_I2C_GPIO_AF              GPIO_AF_I2C1
#define CODEC_I2C_GPIO                 GPIOB
#define CODEC_I2C_SCL_PIN              GPIO_PIN_8
#define CODEC_I2C_SDA_PIN              GPIO_PIN_9
#define CODEC_I2S_SCL_PINSRC           GPIO_PinSource6
#define CODEC_I2S_SDA_PINSRC           GPIO_PinSource7

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define CODEC_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define CODEC_LONG_TIMEOUT             ((uint32_t)(300 * CODEC_FLAG_TIMEOUT))
/*----------------------------------------------------------------------------*/

/*-----------------------------------
                        Audio Codec User defines
                                     -----------------------------------------*/
/* Audio interface : I2S or DAC */
#define AUDIO_INTERFACE_I2S           1
#define AUDIO_INTERFACE_DAC           2



#define OUTPUT_DEVICE_SPEAKER                 ((uint16_t)0x0001)
#define OUTPUT_DEVICE_HEADPHONE               ((uint16_t)0x0002)
#define OUTPUT_DEVICE_BOTH                    ((uint16_t)0x0003)
#define OUTPUT_DEVICE_AUTO                    ((uint16_t)0x0004)


/* Volume Levels values */
#define DEFAULT_VOLMIN                0x00
#define DEFAULT_VOLMAX                0xFF
#define DEFAULT_VOLSTEP               0x04

#define AUDIO_PAUSE                   0
#define AUDIO_RESUME                  1

/* Codec POWER DOWN modes */
#define CODEC_PDWN_HW                 1
#define CODEC_PDWN_SW                 2

/* MUTE commands */
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

#define VOLUME_CONVERT(x)    ((Volume > 100)? 100:((uint8_t)(((uint16_t)(Volume * 255)) / 100)))
#define DMA_MAX(x)           (((x) <= DMA_MAX_SZE)? (x):DMA_MAX_SZE)
#define DMA_MAX_SZE                              ((uint16_t)0xFFFF)
#define FRERAD 8000 

uint32_t AUDIO_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
uint32_t AUDIO_DeInit(void);
uint32_t AUDIO_Play(uint16_t* pBuffer, uint32_t Size);
uint32_t AUDIO_PauseResume(uint32_t Cmd);
uint32_t AUDIO_Stop(uint32_t CodecPowerDown_Mode);
uint32_t AUDIO_VolumeCtl(uint8_t Volume);
uint32_t AUDIO_Mute(uint32_t Command);
void Audio_MAL_Play(uint32_t Addr, uint16_t Size);
void DAC_Config(void);
void codec_sendBeep(void);
void   Audio_MAL_Stop(void);;
void Codec_CtrlInterface_Init(void);
uint32_t Codec_ReadRegister(uint8_t RegisterAddr);
void Codec_GPIO_Init(void);
void AudioUSBSend(uint16_t idxFrm);
void AudioPlayerUpd(void);


/* User Callbacks: user has to implement these functions in his code if
  they are needed. -----------------------------------------------------------*/

uint16_t AUDIO_GetSampleCallBack(void);

/* This function is called when the requested data has been completely transferred.
   In Normal mode (when  the define AUDIO_MAL_MODE_NORMAL is enabled) this function
   is called at the end of the whole audio file.
   In circular mode (when  the define AUDIO_MAL_MODE_CIRCULAR is enabled) this 
   function is called at the end of the current buffer transmission. */
void AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size);

/* This function is called when half of the requested buffer has been transferred 
   This callback is useful in Circular mode only (when AUDIO_MAL_MODE_CIRCULAR 
   define is enabled)*/
void AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void AUDIO_Error_CallBack(void* pData);

/* Codec_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occurs during communication (waiting on an event that doesn't occur, bus 
   errors, busy devices ...) on the Codec control interface (I2C).
   You can use the default timeout callback implementation by uncommenting the 
   define USE_DEFAULT_TIMEOUT_CALLBACK in stm32f4_discovery_audio_codec.h file.
   Typically the user implementation of this callback should reset I2C peripheral
   and re-initialize communication or in worst case reset all the application. */
uint32_t Codec_TIMEOUT_UserCallback(void);

void DMA1_Stream7_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);

//void I2S3_Init(uint32_t AudioFreq);
void Audio_MAL_Play(uint32_t Addr, uint16_t Size);

int WavePlayerInit(uint32_t AudioFreq);



#endif /* __STM32F4_DISCOVERY_AUDIOCODEC_H */
