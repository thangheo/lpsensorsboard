/**
  
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 32 MHz */
//#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
#define I2C_TIMING      0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */ 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
extern I2C_HandleTypeDef hi2c1;


/* Buffer used for reception */
// uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
// void i2c_init(void)
// {
 
//   /*##-1- Configure the I2C peripheral ######################################*/
//   hi2c1.Instance             = I2Cx;
//   hi2c1.Init.Timing          = I2C_TIMING;
//   hi2c1.Init.OwnAddress1     = I2C_ADDRESS;
//   hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
//   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//   hi2c1.Init.OwnAddress2     = 0xFF;
//   hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//   hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  
//   if(HAL_I2C_Init(&hi2c1) != HAL_OK)
//   {
//     /* Initialization Error */
//     Error_Handler();
//   }

//   /* Enable the Analog I2C Filter */
//   HAL_I2CEx_ConfigAnalogFilter(&hi2c1,I2C_ANALOGFILTER_ENABLE);
// }
void i2c_init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0000090F;
    hi2c1.Init.OwnAddress1 = 0x1;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0xFF;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

// void i2c_send(uint16_t dev_addr,uint8_t* data,uint16_t len){
//   /* While the I2C in reception process, user can transmit data through 
//      "aTxBuffer" buffer */
//   /* Timeout is set to 2S */
//   while(HAL_I2C_Master_Transmit(&hi2c1, dev_addr, (uint8_t*)data, len, 2000)!= HAL_OK)
//   {
//     /* Error_Handler() function is called when Timeout error occurs.
//        When Acknowledge failure occurs (Slave don't acknowledge its address)
//        Master restarts communication */
//     if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
//     {
//       Error_Handler();
//     }
//   }
// }
// void i2c_rx(uint16_t dev_addr,uint8_t* data,uint16_t len){
//   /*##-3- Put I2C peripheral in reception process ############################*/ 
//   /* Timeout is set to 10S */ 
//   while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t) dev_addr, (uint8_t *)data, len, 2000) != HAL_OK)
//   {
//     /* Error_Handler() function is called when Timeout error occurs.
//        When Acknowledge failure occurs (Slave don't acknowledge it's address)
//        Master restarts communication */
//     if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
//     {
//       Error_Handler();
//     }
//   }
// }


/**
  * @brief  I2C error callbacks.
  * @param  hi2c1: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c1)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave don't acknowledge it's address, Master restarts communication.
    * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
    */
  if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Error if LED2 is slowly blinking (1 sec. period) */
//   while(1)
//   {    
//     // BSP_LED_Toggle(LED2); 
//     HAL_Delay(1000);
//   } 
    printf("timeout\n");
  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
