
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVERECORDER_H
#define __WAVERECORDER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported Defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines for the Audio recording process */
#define DEFAULT_TIME_REC                      30  /* Recording time in second (default: 30s) */

#define REC_WAVE_NAME "Wave.wav"

#define REC_SAMPLE_LENGTH   (DEFAULT_TIME_REC * DEFAULT_AUDIO_IN_FREQ * DEFAULT_AUDIO_IN_CHANNEL_NBR * 2)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//AUDIO_ErrorTypeDef AUDIO_REC_Process(void);
//AUDIO_ErrorTypeDef AUDIO_REC_Start(void);
//AUDIO_ErrorTypeDef AUDIO_PLAYER_Init(void);

/* sop1hc */
void Audio_Streaming_Ini(void);
void Audio_Streaming(void);
void mySPI_SendData(uint8_t adress, uint8_t data);
void MIC1TO6_Init(void);
void I2S_Proc(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void SPI3_IRQHandler(void);;
uint8_t StartRecord(void);

void GPIO_CLK_Init(void);


void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void I2S1_Enable(void);
void I2S2_Enable(void);
void I2S3_Enable(void);

uint8_t CheckEnergyEqual(int16_t * Channel_Ref, int16_t * Channel, int16_t len);





#endif /* __WAVERECORDER_H */

