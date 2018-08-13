#ifndef __I2C_H
#define __I2C_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

/* Definition for I2Cx_MASTER clock resources */
#define I2Cx_MASTER                             I2C1
#define RCC_PERIPHCLK_I2Cx_MASTER               RCC_PERIPHCLK_I2C1
#define RCC_I2Cx_MASTER_CLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_PCLK1
#define I2Cx_MASTER_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_MASTER_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_MASTER_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_MASTER_DMA_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()

#define I2Cx_MASTER_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_MASTER_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx_SLAVE Pins */
#define I2Cx_MASTER_SCL_PIN                     GPIO_PIN_8
#define I2Cx_MASTER_SDA_PIN                     GPIO_PIN_9
#define I2Cx_MASTER_SCL_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SDA_GPIO_PORT               GPIOB
#define I2Cx_MASTER_SCL_SDA_AF                  GPIO_AF4_I2C1

/* Definition for I2Cx_MASTER's NVIC */
#define I2Cx_MASTER_EV_IRQn                     I2C1_EV_IRQn
#define I2Cx_MASTER_ER_IRQn                     I2C1_ER_IRQn
#define I2Cx_MASTER_EV_IRQHandler               I2C1_EV_IRQHandler
#define I2Cx_MASTER_ER_IRQHandler               I2C1_ER_IRQHandler

/* Definition for I2Cx_SLAVE's DMA */
#define I2Cx_MASTER_DMA                         DMA1   
#define I2Cx_MASTER_DMA_INSTANCE_TX             DMA1_Stream6
#define I2Cx_MASTER_DMA_INSTANCE_RX             DMA1_Stream0
#define I2Cx_MASTER_DMA_CHANNEL_TX              DMA_CHANNEL_1
#define I2Cx_MASTER_DMA_CHANNEL_RX              DMA_CHANNEL_1

/* Definition for I2Cx_SLAVE's DMA NVIC */
#define I2Cx_MASTER_DMA_TX_IRQn                 DMA1_Stream6_IRQn
#define I2Cx_MASTER_DMA_RX_IRQn                 DMA1_Stream0_IRQn
#define I2Cx_MASTER_DMA_TX_IRQHandler           DMA1_Stream6_IRQHandler
#define I2Cx_MASTER_DMA_RX_IRQHandler           DMA1_Stream0_IRQHandler


/* Definition for I2Cx_CPU clock resources */
#define I2Cx_CPU                              I2C4
#define RCC_PERIPHCLK_I2Cx_CPU                RCC_PERIPHCLK_I2C4
#define RCC_I2Cx_CPU_CLKSOURCE_SYSCLK         RCC_I2C4CLKSOURCE_PCLK1
#define I2Cx_CPU_CLK_ENABLE()                __HAL_RCC_I2C4_CLK_ENABLE()
#define I2Cx_CPU_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()
#define I2Cx_CPU_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE() 

#define I2Cx_CPU_FORCE_RESET()               __HAL_RCC_I2C4_FORCE_RESET()
#define I2Cx_CPU_RELEASE_RESET()             __HAL_RCC_I2C4_RELEASE_RESET()

/* Definition for I2Cx_CPU Pins */
#define I2Cx_CPU_SCL_PIN                     GPIO_PIN_12
#define I2Cx_CPU_SDA_PIN                     GPIO_PIN_13
#define I2Cx_CPU_SCL_GPIO_PORT               GPIOD
#define I2Cx_CPU_SDA_GPIO_PORT               GPIOD
#define I2Cx_CPU_SCL_SDA_AF                  GPIO_AF4_I2C4

/* Definition for I2Cx_SLAVE's NVIC */
#define I2Cx_CPU_EV_IRQn                     I2C4_EV_IRQn
#define I2Cx_CPU_ER_IRQn                     I2C4_ER_IRQn
#define I2Cx_CPU_EV_IRQHandler               I2C4_EV_IRQHandler
#define I2Cx_CPU_ER_IRQHandler               I2C4_ER_IRQHandler

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */

#endif