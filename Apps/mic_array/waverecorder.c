
/*****************************************************************************
  *    Author: Phan Le Son                                                                                                                                 
  *    email: plson03@gmail.com
  *****************************************************************************/



/* Includes ------------------------------------------------------------------*/
#include "waverecorder.h" 
#include "string.h"
//#include "stm32f7xx_hal_spi.h"
#include "stm32f7xx_hal.h"
#include "pdm_filter.h"
#include "DSP.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_PIN_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  1
#define SPI_SCK_SOURCE                    1
#define SPI_SCK_AF                        GPIO_AF5_SPI2

#define SPI_MOSI_PIN                      GPIO_PIN_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 1
#define SPI_MOSI_SOURCE                   1
#define SPI_MOSI_AF                       GPIO_AF5_SPI2


/* sop1hc */
#define SPI1_SCK_PIN                       GPIO_PIN_5
#define SPI1_SCK_GPIO_PORT                 GPIOA
#define SPI1_SCK_GPIO_CLK                  1
#define SPI1_SCK_SOURCE                    1
#define SPI1_SCK_AF                        GPIO_AF5_SPI1

#define SPI1_MOSI_PIN                      GPIO_PIN_7
#define SPI1_MOSI_GPIO_PORT                GPIOA
#define SPI1_MOSI_GPIO_CLK                 1
#define SPI1_MOSI_SOURCE                   1
#define SPI1_MOSI_AF                       GPIO_AF5_SPI1

#define SPI1_MISO_PIN                      GPIO_PIN_6
#define SPI1_MISO_GPIO_PORT                GPIOA
#define SPI1_MISO_GPIO_CLK                 1
#define SPI1_MISO_SOURCE                   1
#define SPI1_MISO_AF                       GPIO_AF5_SPI1


int16_t Frame7Old[SHIFT_CHNNL7];
int16_t Frame8Old[SHIFT_CHNNL8];


uint16_t idxMic8=0;
uint16_t idxMic7=0;

uint16_t volatile cntTransFinish;
uint8_t volume;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern  AUDIO_IN_BufferTypeDef  stkBufferCtlRecIn,stkBuffer1, stkBuffer2;
extern AUDIO_OUT_BufferTypeDef  BufferCtlPlayOut;
extern uint16_t __IO idxSPI5DataBuf1, idxSPI5DataBuf2;
extern WAVE_FormatTypeDef WaveFormat;
extern AUDIO_DEMO_StateMachine AudioDemo;
extern AUDIO_PLAYBACK_StateTypeDef AudioState;
extern __IO uint8_t buffer_switch;

extern SPI_HandleTypeDef hspi4,hspi1;
extern __IO uint16_t  WaveRec_idxSens1,WaveRec_idxSens2;
extern __IO uint16_t  WaveRec_idxSens3,WaveRec_idxSens4;
extern __IO uint16_t  WaveRec_idxSens5,WaveRec_idxSens6;
extern SPI_HandleTypeDef     hspi4;




__IO uint8_t flgRacing;
__IO GPIO_PinState stMIC56=GPIO_PIN_SET;
__IO GPIO_PinState stMIC56Old=GPIO_PIN_SET;
__IO uint8_t Wave_cntClk=3;//3
__IO uint8_t stLR;
__IO uint8_t stLROld=GPIO_PIN_RESET;



SPI_HandleTypeDef hspi1,hspi2;
SPI_HandleTypeDef spi1_ins,spi2_ins;
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi5,hspi6;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef     hdma_spi5_rx,hdma_spi6_rx;


__IO uint16_t idxFrmUSB;


uint16_t *bufPCMSens7;
uint16_t *bufPCMSens8;
__IO uint16_t cntPos;
__IO uint16_t cntPos7;
__IO static uint16_t iBuff;
__IO static uint32_t uwVolume = 70;

__IO int16_t   pPDM2PCM[16];

__IO uint16_t WaveRecord_flgIni;
__IO uint8_t WaveRecord_flgInt;
__IO GPIO_PinState Main_stLR = GPIO_PIN_RESET;
__IO GPIO_PinState Main_stLROld = GPIO_PIN_RESET;

uint16_t vRawSens1,vRawSens2,vRawSens4,vRawSens3,vRawSens5,vRawSens6;  
__IO int16_t SPI4_stNipple;
__IO uint16_t iSDO12,iSDO34,iSDO56;

__IO uint8_t I2S1_stPosShft,I2S2_stPosShft,SPI4_stPosShft;
__IO uint8_t I2S2_stLR;
__IO uint8_t I2S2_stLROld = GPIO_PIN_RESET;

/* Private function prototypes -----------------------------------------------*/
static void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
static uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
static void I2S1_Init(void);
static void I2S2_Init(void);
static void I2S3_Init(void);


#if EXT_RAM
#pragma location=SDRAM_BANK_ADDR
#endif
#pragma data_alignment = 64
Mic_Array_Data Buffer1;

#if EXT_RAM
#pragma location= (SDRAM_BANK_ADDR+ BUFFER_SIZE_BYTE)
#endif
#pragma data_alignment = 64
Mic_Array_Data Buffer2 ;

#if EXT_RAM
#pragma location= (SDRAM_BANK_ADDR+ BUFFER_SIZE_BYTE + BUFFER_SIZE_BYTE)
#endif
#pragma data_alignment = 64
Mic_Array_Data Buffer3;


void mySPI_SendData(uint8_t adress, uint8_t data)
{
 
while(!__HAL_SPI_GET_FLAG(&spi1_ins, SPI_FLAG_TXE)); 
SPI_I2S_SendData(SPI1, adress);

while(!__HAL_SPI_GET_FLAG(&spi1_ins, SPI_FLAG_RXNE));
SPI_I2S_ReceiveData(SPI1);

while(!!__HAL_SPI_GET_FLAG(&spi1_ins, SPI_FLAG_TXE)); 
SPI_I2S_SendData(SPI1, data);

while(!!__HAL_SPI_GET_FLAG(&spi1_ins, SPI_FLAG_RXNE));
SPI_I2S_ReceiveData(SPI1);
 
}

/**
  * @brief  This function handles AUDIO_REC_SPI global interrupt request.
  * @param  None
  * @retval None
*/

void SPI1_IRQHandler(void)
{  
      int16_t tmpTest;
	  
	
	  /* SPI in mode Receiver ----------------------------------------------------*/
	  if(
	     //(__HAL_SPI_GET_FLAG(&hi2s1, SPI_FLAG_OVR) == RESET)&&
	     //(__HAL_SPI_GET_FLAG(&hi2s1, SPI_FLAG_RXNE) != RESET)&&
		 (__HAL_I2S_GET_IT_SOURCE(&hi2s1, SPI_IT_RXNE) != RESET))
	  {
	

	   tmpTest =  (int16_t)SPI_I2S_ReceiveData(SPI1);

        Wave_cntClk++;
        if (Wave_cntClk==4)
        {
            Wave_cntClk = 0;
            stLR = GPIO_PIN_SET;
            I2S2_stLR = GPIO_PIN_SET; 
            Main_stLR = GPIO_PIN_SET;
        }
        else if (Wave_cntClk==1)
        {
            stLR = GPIO_PIN_SET;
            I2S2_stLR = GPIO_PIN_SET;
            Main_stLR = GPIO_PIN_SET;
        }
        else
        {
            stLR = GPIO_PIN_RESET;
            I2S2_stLR = GPIO_PIN_RESET;
            Main_stLR = GPIO_PIN_RESET;
        }
       

	   /* Left-Right Mic data */
	   //stLR= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	
		if (stLR==GPIO_PIN_SET)
		{
		       if(stLROld==GPIO_PIN_SET) 
		       {
				   vRawSens1 = (tmpTest);
				   /* Recording Audio Data */						 
					if (WaveRec_idxSens1<PAR_N) 
					{
					   switch (buffer_switch)
					   {
                              case BUF1_PLAY:
                                  Buffer2.bufMIC1[WaveRec_idxSens1] = vRawSens1;//vRawSens1;								
                                  break;
                              case BUF2_PLAY:
                                  Buffer3.bufMIC1[WaveRec_idxSens1] = vRawSens1;//vRawSens1;
                                  break;
                              case BUF3_PLAY:
                                  Buffer1.bufMIC1[WaveRec_idxSens1] = vRawSens1;//vRawSens1;									
                                  break;
                              default:
                                  break; 
					   
					   }
					  
					}

					WaveRec_idxSens1++;
                    if ((WaveRec_idxSens1 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x01;
			        if (flgRacing==0x3F)  SubFrameFinished();                    
		       	}
		}
		else
		{		
          if ((stLROld==GPIO_PIN_RESET))  
          {
		      vRawSens2 = (tmpTest);
			  if (WaveRec_idxSens2<PAR_N)
			  {
					/* Recording Audio Data */						 
					switch (buffer_switch)
					{
						case BUF1_PLAY:
							Buffer2.bufMIC2[WaveRec_idxSens2] = vRawSens2;								
							break;
						case BUF2_PLAY:
							Buffer3.bufMIC2[WaveRec_idxSens2] = vRawSens2;
							break;
						case BUF3_PLAY:
							Buffer1.bufMIC2[WaveRec_idxSens2] = vRawSens2;									
							break;
						default:
							break; 

				    }
					
              }
			 	
  	          WaveRec_idxSens2++;
		      if ((WaveRec_idxSens2 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x02;
		      if (flgRacing==0x3F)  SubFrameFinished();			      					
		    }
		
	    } 	

        /* Update Old value */    
        stLROld=stLR;

		
    }
	   

				 
} 	 




/**
  * @brief  This function handles AUDIO_REC_SPI global interrupt request.
  * @param  None
  * @retval None
*/

void SPI2_IRQHandler(void)
{      
    int16_t app;
    

  /* Check if data are available in SPI Data register */
   if (
	   //(__HAL_SPI_GET_FLAG(&hi2s2, SPI_FLAG_OVR) == RESET)&&
   	   // (__HAL_SPI_GET_FLAG(&hi2s2, SPI_FLAG_RXNE) != RESET)&&
   	    (__HAL_I2S_GET_IT_SOURCE(&hi2s2, SPI_IT_RXNE)!=RESET)
   	  )
   {
    
     app = (int16_t)SPI_I2S_ReceiveData(SPI2);   
     //SPI_I2S_SendData(SPI2, 3333);

	 //I2S2_stLR= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
     
	 if (I2S2_stLR==GPIO_PIN_RESET)
	 {		
		if ((I2S2_stLROld==GPIO_PIN_RESET)) 
		{
			vRawSens4 = app;
	
			if (WaveRec_idxSens4< PAR_N)
			{
				switch (buffer_switch)
				{
					case BUF1_PLAY:
						Buffer2.bufMIC4[WaveRec_idxSens4] = vRawSens4;								
						break;
					case BUF2_PLAY:
						Buffer3.bufMIC4[WaveRec_idxSens4] = vRawSens4;
						break;
					case BUF3_PLAY:
						Buffer1.bufMIC4[WaveRec_idxSens4] = vRawSens4;									
						break;
					default:
						break; 
				}
				
			}									

		    WaveRec_idxSens4++;

			if ((WaveRec_idxSens4 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x08;

			if (flgRacing==0x3F)  SubFrameFinished();
			
		}
		
	 }
	 else
	 {
            if ((I2S2_stLROld==GPIO_PIN_SET))   
            {
                vRawSens3 =app;
                if ((WaveRec_idxSens3<PAR_N))
                {
	                switch (buffer_switch)
	                {	 
                      case BUF1_PLAY:
                          Buffer2.bufMIC3[WaveRec_idxSens3] = vRawSens3;								
                          break;
                      case BUF2_PLAY:
                          Buffer3.bufMIC3[WaveRec_idxSens3] = vRawSens3;
                          break;
                      case BUF3_PLAY:
                          Buffer1.bufMIC3[WaveRec_idxSens3] = vRawSens3;									
                          break;
                      default:
                          break; 
	                }						
                }
                WaveRec_idxSens3++;
                if ((WaveRec_idxSens3 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x04;
                if (flgRacing==0x3F)  SubFrameFinished();				  
                    
            }
	    }//else	  
	    I2S2_stLROld = I2S2_stLR;
   }

}


void SPI3_IRQHandler(void)
{
  uint16_t test;
  /* SPI in mode Receiver ----------------------------------------------------*/
  if(
  //  (__HAL_SPI_GET_FLAG(&hi2s3, SPI_FLAG_OVR) == RESET)&&
  //  (__HAL_SPI_GET_FLAG(&hi2s3, SPI_FLAG_RXNE) != RESET)&&
     (__HAL_SPI_GET_IT_SOURCE(&hi2s3, SPI_IT_RXNE) != RESET))
  {

    
    test =  SPI_I2S_ReceiveData(SPI3);

  
	if (Main_stLR==GPIO_PIN_RESET)
	{
          if (Main_stLROld==GPIO_PIN_RESET)
          {

             vRawSens6 =test;

             if (WaveRec_idxSens6 < PAR_N)
             {
                  /*-------------------------------------------------------------------------------------------------------------                                             
                  Sequence  Record Data                     Processing Data                 Player Data
                                    
                  1-------  Buffer1                         Buffer2                         Buffer3 BUF3_PLAY)
                                    
                  2-------  Buffer3                         Buffer1                         Buffer2 (BUF2_PLAY)		  
                                    
                  3-------  Buffer2                         Buffer3                         Buffer1 (BUF1_PLAY)
                   ---------------------------------------------------------------------------------------------------------------*/                     
                   /* Recording Audio Data */			             
                   switch (buffer_switch)
                   {
                       case BUF1_PLAY:
                           Buffer2.bufMIC6[WaveRec_idxSens6] =vRawSens6;

                           break;
                       case BUF2_PLAY:
                           Buffer3.bufMIC6[WaveRec_idxSens6] =vRawSens6;

                           break;
                       case BUF3_PLAY:
                           Buffer1.bufMIC6[WaveRec_idxSens6] =vRawSens6;

                           break;                          
                       default:
                           break;
                   }		
	           }
               WaveRec_idxSens6++;                     
               if ((WaveRec_idxSens6 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x20;
               if (flgRacing==0x3F)  SubFrameFinished();
       }
          
    }
	else
	{    
          if (Main_stLROld==GPIO_PIN_SET)
          {
               vRawSens5 =test;
               if (WaveRec_idxSens5 < PAR_N)
               {
                   /*-------------------------------------------------------------------------------------------------------------                                             
                   Sequence  Record Data                     Processing Data                 Player Data
                                      
                   1-------  Buffer1                         Buffer2                         Buffer3 BUF3_PLAY)
                                      
                   2-------  Buffer3                         Buffer1                         Buffer2 (BUF2_PLAY)		  
                                      
                   3-------  Buffer2                         Buffer3                         Buffer1 (BUF1_PLAY)
                   ---------------------------------------------------------------------------------------------------------------*/                 
                     /* Recording Audio Data */			             
                     switch (buffer_switch)
                     {
                         case BUF1_PLAY:
                             Buffer2.bufMIC5[WaveRec_idxSens5] = vRawSens5;

                             break;
                         case BUF2_PLAY:
                             Buffer3.bufMIC5[WaveRec_idxSens5] = vRawSens5;
       
                             break;
                         case BUF3_PLAY:
                             Buffer1.bufMIC5[WaveRec_idxSens5] = vRawSens5;

                             break;                          
                         default:
                             break;
                     }	  

               }
                
               //if ((  WaveRec_idxSens5 - WaveRec_idxSens1 > 0)&&(WaveRec_idxSens5==10))
               //{  
               //    HAL_NVIC_SystemReset();
               //} 

             WaveRec_idxSens5++;
             if ((WaveRec_idxSens5 % (AUDIO_SAMPLING_FREQUENCY/1000)==0)) flgRacing |=0x10;
             if (flgRacing==0x3F)  SubFrameFinished();
          }    
	}
	/* Update Old value */	  
	Main_stLROld=Main_stLR;	  
  }      
}


void MIC1TO6_Init(void)
{

  I2S1_Init(); /* I2S1   --> SDO12 */

  I2S2_Init(); /* I2S2   --> SDO34 */

  I2S3_Init(); /* I2S3   --> SDO56 */


}



/* I2S1 init function */
/* Read data of MIC12 */
static void I2S1_Init(void)
{
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s1.Init.Standard = I2S_STANDARD_LSB;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_SYSCLK;
  HAL_I2S_Init(&hi2s1);
}




/* I2S2 init function */
/* Read data of MIC34 */

static void I2S2_Init(void)
{

 //HAL_I2S_DeInit(&hi2s2);
 hi2s2.Instance = SPI2;
 hi2s2.Init.Mode = I2S_MODE_SLAVE_RX;//I2S_MODE_MASTER_RX
 hi2s2.Init.Standard = I2S_STANDARD_LSB;//I2S_STANDARD_LSB
 hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
 hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
 hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
 hi2s2.Init.CPOL = I2S_CPOL_LOW;
 hi2s2.Init.ClockSource = I2S_CLOCK_SYSCLK;

 HAL_I2S_Init(&hi2s2);

}

static void I2S3_Init(void)
{

  // static I2S_HandleTypeDef hi2s3;
  /* Enable the CODEC_I2S peripheral clock */
  __HAL_RCC_SPI3_CLK_ENABLE();

  hi2s3.Instance = SPI3;
  /* Disable I2S3 peripheral to allow access to I2S internal registers */
  //__HAL_I2S_DISABLE(&hi2s3);
  
  hi2s3.Init.Standard = I2S_STANDARD_LSB;//I2S_STANDARD_PHILIPS
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_SYSCLK;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;

#ifdef CODEC_MCLK_ENABLED
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
#elif defined(CODEC_MCLK_DISABLED)
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
#else
#error "No selection for the MCLK output has been defined !"
#endif /* CODEC_MCLK_ENABLED */
  
  /* Initialize the I2S peripheral with the structure above */
  HAL_I2S_Init(&hi2s3);


}

void I2S1_Enable(void)
{
   /* Enable TXE and ERR interrupt */
 __HAL_I2S_ENABLE_IT(&hi2s1, (I2S_IT_RXNE));
 __HAL_I2S_ENABLE(&hi2s1);
}


void I2S2_Enable(void)
{
    /* Enable TXE and ERR interrupt */
    __HAL_I2S_ENABLE_IT(&hi2s2, (I2S_IT_RXNE));
    __HAL_I2S_ENABLE(&hi2s2);
}

void I2S3_Enable(void)
{
    /* Enable TXE and ERR interrupt */
    __HAL_I2S_ENABLE_IT(&hi2s3, (I2S_IT_RXNE));
    __HAL_I2S_ENABLE(&hi2s3);
}


uint8_t StartRecord(void)
{           
   I2S1_Enable();
   I2S2_Enable();
   I2S3_Enable();
   
   WaveRec_idxSens1 = 0;
   WaveRec_idxSens2 = 0;
   WaveRec_idxSens3 = 0;
   WaveRec_idxSens4 = 0;
   WaveRec_idxSens5 = 0;
   WaveRec_idxSens6 = 0; 

   idxFrmUSB = 0;
 
   buffer_switch = BUF1_PLAY;

   return 0;
    
}



static void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
  
  /* Write in the DR register the data to be sent */
  SPIx->DR = Data;
}


static uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
  
  /* Return the data in the DR register */
  return SPIx->DR;
}







