#ifndef __WS281X_H
#define __WS281X_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* Definition for TIM instance */
#define TIMx                        TIM1

/* Definition for TIMx clock resources */
#define TIMx_CLK_ENABLE()                   __HAL_RCC_TIM1_CLK_ENABLE()
#define PWM_DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()

/* Definition for TIMx Pins */
#define TIMx_CHANNEL1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define TIMx_GPIO_CHANNEL1_PORT             GPIOA
#define TIMx_GPIO_CHANNEL1_PIN              GPIO_PIN_10
#define TIMx_GPIO_AF_CHANNEL1               GPIO_AF1_TIM1

/* Definition for TIMx's DMA */
#define TIMx_CC1_DMA_INST                DMA2_Stream6
#define TIMx_DMA_CHANNEL_REQUEST         DMA_CHANNEL_6
/* Definition for ADCx's NVIC */
#define TIMx_DMA_IRQn                    DMA2_Stream6_IRQn
#define TIMx_DMA_IRQHandler              DMA2_Stream6_IRQHandler

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                         TIM1_CC_IRQn
  /* ---------------------------------------------------------------------------

  TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1)x2, since APB1
  prescaler is 4.
    TIM2CLK = PCLK1*2
    PCLK1 = HCLK/2
    => TIM2CLK = HCLK/2 = SystemCoreClock/2

  TIM2CLK = (SystemCoreClock/2), Prescaler = 0, TIM2 counter clock = (SystemCoreClock/2)
  SystemCoreClock is set to 216 MHz for STM32F7xx devices.
*/
//WS2812
#define WS2812_FREQ                                         (800000)            // it is fixed: WS2812 require 800kHz
#define TIMER_CLOCK_FREQ                                    SystemCoreClock      // can be modified - multiples of 0.8MHz are suggested
#define TIMER_PERIOD                                        (TIMER_CLOCK_FREQ / WS2812_FREQ)
#define LED_NUMBER                                          (16)            // how many LEDs the MCU should control?
#define LED_DATA_SIZE                                       (LED_NUMBER * 24)
#define RESET_SLOTS_BEGIN                                   (50)
#define RESET_SLOTS_END                                     (50)
#define WS2812_LAST_SLOT                                    (1)
#define LED_BUFFER_SIZE                                     (RESET_SLOTS_BEGIN + LED_DATA_SIZE + WS2812_LAST_SLOT + RESET_SLOTS_END)
#define WS2812_0                                            (TIMER_PERIOD / 3)              // WS2812's zero high time is long about one third of the period
#define WS2812_1                                            (TIMER_PERIOD * 2 / 3)      // WS2812's one high time is long about two thirds of the period
#define WS2812_RESET                                        (0)

void ws281x_init(void);
void ws281x_update(void);
void setLEDcolor(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE);
void setWHOLEcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE);
void fillBufferBlack(void);
void fillBufferWhite(void);
void TIMx_DMA_IRQHandler(void);

#endif