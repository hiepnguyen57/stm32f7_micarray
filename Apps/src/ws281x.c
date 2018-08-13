#include "ws281x.h"
#include <stdio.h>
#include <string.h>
/* Variables -----------------------------------------------*/
static uint32_t LEDbuffer[LED_BUFFER_SIZE];

TIM_HandleTypeDef       TimHandle;
TIM_OC_InitTypeDef      sConfig;
GPIO_InitTypeDef        GPIO_InitStruct;
DMA_HandleTypeDef       hdma_tim;

/* Functions -----------------------------------------------*/

/**
 * @brief TIM MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - DMA configuration for transmission request by peripheral
 * @param htim: TIM handle pointer
 * @retval None
 */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx clock enable */
	TIMx_CLK_ENABLE();

	/* Enable GPIO Channel Clock */
	TIMx_CHANNEL1_GPIO_CLK_ENABLE();

	/* Enable DMA clock */
	PWM_DMAx_CLK_ENABLE();

	/* Configure TIM1 channel 1 in output, push-pull & alternate function mode */
	GPIO_InitStruct.Pin = TIMx_GPIO_CHANNEL1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = TIMx_GPIO_AF_CHANNEL1;
	HAL_GPIO_Init(TIMx_GPIO_CHANNEL1_PORT, &GPIO_InitStruct);

	/* Set the parameters to be configured */
	hdma_tim.Init.Channel = TIMx_DMA_CHANNEL_REQUEST;
	hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
	hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
	hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
	hdma_tim.Init.Mode = DMA_CIRCULAR;
	hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

	/* Set hdma_tim instance */
	hdma_tim.Instance = TIMx_CC1_DMA_INST;

	/* Link DMA tim to hdma[TIM_DMA_ID_CC1] (Channel1) */
	__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC3], hdma_tim);

	/* Initialize TIMx DMA handle*/
	HAL_DMA_Init(htim->hdma[TIM_DMA_ID_CC3]);

	/* Configure the NVIC for DMA */
	/* NVIC configuration for DMA transfer complete interrupt */
	HAL_NVIC_SetPriority(TIMx_DMA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIMx_DMA_IRQn);
}

void ws281x_init(void)
{
	fillBufferBlack();

	TimHandle.Instance = TIMx;

	TimHandle.Init.Period 				= TIMER_PERIOD - 1;
	TimHandle.Init.RepetitionCounter 	= 0;
	TimHandle.Init.Prescaler			= 0;
	TimHandle.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode 			= TIM_COUNTERMODE_UP;
	TimHandle.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(&TimHandle);

	/* Configure the PWM channel 1 */
	sConfig.OCMode 		 = TIM_OCMODE_PWM2;
	sConfig.OCPolarity 	 = TIM_OCPOLARITY_LOW;
//	sConfig.Pulse        = 99;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState  = TIM_OCNIDLESTATE_SET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3);

	/*Start PWM signal generation in DMA mode ############################*/
	HAL_TIM_PWM_Start_DMA(&TimHandle, TIM_CHANNEL_3, LEDbuffer,
			LED_BUFFER_SIZE);
}

void ws281x_update(void)
{
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_DMA(&TimHandle, TIM_CHANNEL_3, LEDbuffer,
			LED_BUFFER_SIZE);
}

void setLEDcolor(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE)
{
	uint32_t tempBuffer[24];
	uint32_t i;
	uint32_t LEDindex;
	LEDindex = LEDnumber % LED_NUMBER;

	for (i = 0; i < 8; i++) // GREEN data
		tempBuffer[i] = ((GREEN << i) & 0x80) ? WS2812_1 : WS2812_0;
	for (i = 0; i < 8; i++) // RED
		tempBuffer[8 + i] = ((RED << i) & 0x80) ? WS2812_1 : WS2812_0;
	for (i = 0; i < 8; i++) // BLUE
		tempBuffer[16 + i] = ((BLUE << i) & 0x80) ? WS2812_1 : WS2812_0;

	for (i = 0; i < 24; i++)
		LEDbuffer[RESET_SLOTS_BEGIN + LEDindex * 24 + i] = tempBuffer[i];
}

void setWHOLEcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE)
{
	uint32_t index;

	for (index = 0; index < LED_NUMBER; index++)
		setLEDcolor(index, RED, GREEN, BLUE);
}

void fillBufferBlack(void)
{
	/*Fill LED buffer - ALL OFF*/
	uint32_t index, buffIndex;
	buffIndex = 0;

	for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
		LEDbuffer[buffIndex] = WS2812_RESET;
		buffIndex++;
	}
	for (index = 0; index < LED_DATA_SIZE; index++) {
		LEDbuffer[buffIndex] = WS2812_0;
		buffIndex++;
	}
	LEDbuffer[buffIndex] = WS2812_0;
	buffIndex++;
	for (index = 0; index < RESET_SLOTS_END; index++) {
		LEDbuffer[buffIndex] = 0;
		buffIndex++;
	}
}

void fillBufferWhite(void)
{
	/*Fill LED buffer - ALL OFF*/
	uint32_t index, buffIndex;
	buffIndex = 0;

	for (index = 0; index < RESET_SLOTS_BEGIN; index++) {
		LEDbuffer[buffIndex] = WS2812_RESET;
		buffIndex++;
	}
	for (index = 0; index < LED_DATA_SIZE; index++) {
		LEDbuffer[buffIndex] = WS2812_1;
		buffIndex++;
	}
	LEDbuffer[buffIndex] = WS2812_0;
	buffIndex++;
	for (index = 0; index < RESET_SLOTS_END; index++) {
		LEDbuffer[buffIndex] = 0;
		buffIndex++;
	}
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_DMA_IRQHandler(void)
{
	HAL_DMA_IRQHandler(TimHandle.hdma[TIM_DMA_ID_CC3]);
}