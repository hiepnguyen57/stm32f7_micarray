#include "log.h"
#include "main.h"
UART_HandleTypeDef huart1;

void log_init(void)
{
    huart1.Instance        = USARTx;

    huart1.Init.BaudRate   = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
    if(HAL_UART_DeInit(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }  
    if(HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

int _write(int fd, char * str, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, len , 100);
    return len;
}

