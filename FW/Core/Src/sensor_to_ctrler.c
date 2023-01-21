#include <stdio.h>
#include "stm32l0xx_hal.h"
#include "app.h"
extern UART_HandleTypeDef huart1;
__IO ITStatus UartReady = RESET;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
}
void tx_to_controller(void* data)
{
    /*##-2- Start the transmission process #####################################*/  
    /* While the UART in reception process, user can transmit data through 
        "aTxBuffer" buffer */
    if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, sizeof(sensor_data_fusion_t))!= HAL_OK)
    {
        Error_Handler();
    }
    
    /*##-3- Wait for the end of the transfer ###################################*/   
    while (UartReady != SET)
    {
    }
    /* Reset transmission flag */
    UartReady = RESET;

}
void rx_from_controller(void* data){

    /* Reset transmission flag */
    UartReady = RESET;
    
    /*##-4- Put UART peripheral in reception process ###########################*/  
    if(HAL_UART_Receive_IT(&huart1, (uint8_t *)data, sizeof(sensor_data_fusion_t)) != HAL_OK)
    {
        Error_Handler();
    }
    while (UartReady != SET)
    {
    }
    /* Reset transmission flag */
    UartReady = RESET;

}
 