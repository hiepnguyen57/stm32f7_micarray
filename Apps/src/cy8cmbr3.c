#include <stdio.h>
#include <stdlib.h>
#include "cy8cmbr3.h"

/* Above are the Command Codes used to configure MBR3*/
uint8_t configData[129] = {
    //The below configuration array enables all 4 buttons, Host interrupt
    0x00u, //REMAP_REGISTER
    0x78u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
    0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
    0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
    0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
    0x00u, 0x0Fu, 0xFFu, 0xFFu, 0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu,
    0xFFu, 0x00u, 0x00u, 0x00u, 0x04u, 0x03u, 0x01u, 0x58u,
    0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x1Au, 0x5Fu
};

// uint8_t configData[129] = {
//     //The below configuration array enables all 4 buttons, Host INT, Proximity
//     0x00u, // REMAP_REGISTER
//     0x78u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
//     0x80u, 0x80u, 0x80u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
//     0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
//     0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
//     0x00u, 0x0Fu, 0xFFu, 0xFFu, 0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu,
//     0xFFu, 0x00u, 0x00u, 0x00u, 0x24u, 0x03u, 0x01u, 0x59u,
//     0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
//     0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x7Au, 0xFCu
// };

uint8_t *TxBuffer;

void MBR3_HOST_INT_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB0 pin as input floating */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable adn set EXTI Line0 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

CY8CMBR3116_Result ConfigureMBR3(I2C_HandleTypeDef *I2Cx)
{
    uint8_t REGMAP_REG = (uint8_t)REGMAP_ORIGIN;
    I2C_HandleTypeDef* handle = I2Cx;

    //Wake up the MBR3 device
    /* fix me, following cypress document, MBR3 need to be wrote
        data more than 3 times to wake up */ 
    if(HAL_I2C_IsDeviceReady(handle, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
    {
        logs_error("CY3280-MBR3 is not ready");
    }

    // send configuration
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)configData, 129, 1000) != HAL_OK)
    {
        logs_error("Send configuration");
        return CY8CMBR3116_Result_ERROR;
    }
//    HAL_Delay(100); //need, if dont run

    //apply configuration to MBR3
    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SAVE_CHECK_CRC;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
    {
        logs_error("CRC checking");
        return CY8CMBR3116_Result_ERROR;
    }
    HAL_Delay(100);

    TxBuffer[0] = CTRL_CMD;
    TxBuffer[1] = SW_RESET;
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                (uint8_t *)TxBuffer, 2, 1000) != HAL_OK)
    {
        logs_error("Reset SW");
        return CY8CMBR3116_Result_ERROR;
    }

    //Provide a delay to calculate and save CRC
    HAL_Delay(200);
    return CY8CMBR3116_Result_OK;
}

CY8CMBR3116_Result ReadandDisplaySensorStatus(I2C_HandleTypeDef *I2Cx)
{
    uint8_t button_stat;
    uint8_t BUTTON_REG = (uint8_t)BUTTON_STATUS;
    I2C_HandleTypeDef* handle = I2Cx;

    /* fix me, following cypress document, MBR3 need to be wrote
        data more than 3 times to wake up */ 
    if(HAL_I2C_IsDeviceReady(handle, (uint16_t)MBR3_ADDR, 4, 5) != HAL_OK)
    {
        logs_error("CY3280-MBR3 is not ready");
    }
    //Config button
    if (HAL_I2C_Master_Transmit(handle, (uint16_t)MBR3_ADDR, 
                &BUTTON_REG, 1, 1000) != HAL_OK)
    {
        logs_error("Configure button");
        return CY8CMBR3116_Result_ERROR;
    }

    if(HAL_I2C_Master_Receive(handle, (uint16_t)MBR3_ADDR,
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
    static int touched=0, prox=0;

    if(touched)
    {
        touched = 0;
        logs("Button released");
    }

    if(buffer & BUTTON_1)
    {
        //do something
        logs("Button 1 TOUCHED");
        touched = 1;
    }

    if((buffer & BUTTON_2))
    {
        logs("Button 2 TOUCHED");
        touched = 1;
    }

    if(buffer & BUTTON_3)
    {
        logs("Button 3 TOUCHED");
        touched = 1;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if(buffer & BUTTON_4)
    {
        logs("Button 4 TOUCHED");
        touched = 1;
        //define I2C4 send here
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

    }
}