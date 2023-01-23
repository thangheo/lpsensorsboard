#include "app.h"
#include "stm32l0xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h" 
#include "event_groups.h"
// uint32_t u32_sensor_data_ready=0;
//function prototype;
//global vars
extern SemaphoreHandle_t ADC1_Mutex;
extern SemaphoreHandle_t I2C1_Mutex;
extern UART_HandleTypeDef huart1;
extern EventGroupHandle_t xSensorsEventGroup;
__IO ITStatus UartReady = RESET;

void power_saving_task(void *pvParameters){
    //wait something here
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    for(;;){
        //turn off sensor power supply before go to sleep for the next incarnation
        EventBits_t uxBits;
        const TickType_t xTicksToWait = 150000 / portTICK_PERIOD_MS;

        /* Wait a maximum of 120s for either bit 0 or bit 4 to be set within
        the event group.  Clear the bits before exiting. */
        uxBits = xEventGroupWaitBits(
                    xSensorsEventGroup,  
                    b_GOTO_SLEEP_FOREVER, 
                    pdTRUE,       
                    pdFALSE,       /* Don't wait for both bits, either bit will do. */
                    xTicksToWait );
        
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_PWR_EnterSTANDBYMode();
    }
}
void read_light_sensor_task(void *pvParameters){
    for(;;){
        xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);

        float Vpr=0;
        sensor_data_fusion_t *data = pvParameters;

        ADC_HandleTypeDef hadc;
        ADC_ChannelConfTypeDef sConfig = {0};

        /* USER CODE BEGIN ADC_Init 1 */

        /* USER CODE END ADC_Init 1 */

        /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
         */
        hadc.Instance = ADC1;
        hadc.Init.OversamplingMode = DISABLE;
        hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
        hadc.Init.Resolution = ADC_RESOLUTION_12B;
        hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
        hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc.Init.ContinuousConvMode = DISABLE;
        hadc.Init.DiscontinuousConvMode = DISABLE;
        hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc.Init.DMAContinuousRequests = DISABLE;
        hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
        hadc.Init.LowPowerAutoWait = DISABLE;
        hadc.Init.LowPowerFrequencyMode = DISABLE;
        hadc.Init.LowPowerAutoPowerOff = DISABLE;
        if (HAL_ADC_Init(&hadc) != HAL_OK)
        {
            Error_Handler();
        }
        /* Select Channel 0 to be converted 
        channel 0 -->Vpr
        channel 1 --> MICrophone
        */
        sConfig.Channel = ADC_CHANNEL_1;    
        if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        
        /*##- 4- Start the conversion process #######################################*/  
        for(int idx=0;idx < 10;idx++){
            if(HAL_ADC_Start(&hadc) != HAL_OK)
            {
                /* Start Conversation Error */
                Error_Handler();
            }
        
            /*##- 5- Wait for the end of conversion #####################################*/  
            /*  Before starting a new conversion, you need to check the current state of
                the peripheral; if it’s busy you need to wait for the end of current
                conversion before starting a new one.
                For simplicity reasons, this example is just waiting till the end of the
                conversion, but application may perform other tasks while conversion
                operation is ongoing. */
            HAL_ADC_PollForConversion(&hadc, 10);
        
            /* Check if the continous conversion of regular channel is finished */
            if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
            {
                /*##-6- Get the converted value of regular channel  ########################*/
                Vpr += HAL_ADC_GetValue(&hadc);
            }
            //sleep 100ms
            const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
            vTaskDelay( xDelay );
            // (float)Lux = ((float) Vpr)/817 - 1.708796
        }
        //release ADC mutex
        xSemaphoreGive(ADC1_Mutex);

        Vpr /=10;
        data->light = Vpr*3 -1;
        
        //sleep for 60s
        const TickType_t xDelay = 60000 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );

    }
}


void read_AM2320_hu_temp_data(void *pvParameters){
    for(;;){
        xSemaphoreTake(I2C1_Mutex, portMAX_DELAY);

        sensor_data_fusion_t *data = pvParameters;
        // Returns number of bytes written
        Am2320_HandleTypeDef Am2320_;
        I2C_HandleTypeDef hi2c1;

        Am2320_ = am2320_Init(&hi2c1, AM2320_ADDRESS);
        float temperature, humidity;
        /* USER CODE END 2 */
        /* Infinite loop */
        /* USER CODE BEGIN WHILE */

        am2320_GetTemperatureAndHumidity(&Am2320_, &temperature, &humidity);
        data->AM2320_humidity= (uint32_t) humidity*100;
        data->AM2320_temperature= (uint32_t) temperature*100;
        xSemaphoreGive(I2C1_Mutex);

        //data ready
        // u32_sensor_data_ready |= bAM2320_BIT; 
        xEventGroupSetBits(
                            xSensorsEventGroup,    
                            bAM2320_BIT );
        // my_printf("Temperature: %.1fºC\r\n", temperature);
        // my_printf("Humidity: %.1f%%\r\n", humidity);
            /* USER CODE END WHILE */
            //sleep endless time
        const TickType_t xDelay = portMAX_DELAY / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
    }
}

void read_audio_data_task(void *pvParameters){
    for(;;){
        sensor_data_fusion_t *data = pvParameters;
        xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);

        int adc_val=0;
        ADC_HandleTypeDef hadc;

        ADC_ChannelConfTypeDef sConfig = {0};

        /* USER CODE BEGIN ADC_Init 1 */

        /* USER CODE END ADC_Init 1 */

        /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
         */
        hadc.Instance = ADC1;
        hadc.Init.OversamplingMode = DISABLE;
        hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
        hadc.Init.Resolution = ADC_RESOLUTION_12B;
        hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
        hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc.Init.ContinuousConvMode = DISABLE;
        hadc.Init.DiscontinuousConvMode = DISABLE;
        hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc.Init.DMAContinuousRequests = DISABLE;
        hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
        hadc.Init.LowPowerAutoWait = DISABLE;
        hadc.Init.LowPowerFrequencyMode = DISABLE;
        hadc.Init.LowPowerAutoPowerOff = DISABLE;
        if (HAL_ADC_Init(&hadc) != HAL_OK)
        {
            Error_Handler();
        }
        /* Select Channel 0 to be converted 
        channel 0 -->Vpr
        channel 1 --> MICrophone
        */
        sConfig.Channel = ADC_CHANNEL_2;    
        if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        
        /*##- 4- Start the conversion process #######################################*/  
        if(HAL_ADC_Start(&hadc) != HAL_OK)
        {
            /* Start Conversation Error */
            Error_Handler();
        }
    
        /*##- 5- Wait for the end of conversion #####################################*/  
        /*  Before starting a new conversion, you need to check the current state of
            the peripheral; if it’s busy you need to wait for the end of current
            conversion before starting a new one.
            For simplicity reasons, this example is just waiting till the end of the
            conversion, but application may perform other tasks while conversion
            operation is ongoing. */
        HAL_ADC_PollForConversion(&hadc, 10);
    
        /* Check if the continous conversion of regular channel is finished */
        if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
        {
        /*##-6- Get the converted value of regular channel  ########################*/
        adc_val = HAL_ADC_GetValue(&hadc);
        }
         //release ADC mutex
        xSemaphoreGive(ADC1_Mutex);

        //  Vpr=0.33*(1+L_i)
        // -->Li = Vpr*3-1 (Lux) (min)
        data->audio = adc_val;
        // u32_sensor_data_ready |= b_audio_BIT;
        xEventGroupSetBits(
                            xSensorsEventGroup,    
                            b_audio_BIT );
        const TickType_t xDelay = portMAX_DELAY / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
    }
}

void read_sensirion_data_task(void *pvParameters) {
    for(;;){
        sensor_data_fusion_t *data = pvParameters;
        int16_t error = 0;
        xSemaphoreTake(I2C1_Mutex, portMAX_DELAY);

        sensirion_i2c_hal_init();

        // Clean up potential SCD40 states
        scd4x_wake_up();
        scd4x_stop_periodic_measurement();
        scd4x_reinit();

        // uint16_t serial_0;
        // uint16_t serial_1;
        // uint16_t serial_2;
        // error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
        // if (error) {
        //     printf("Error executing scd4x_get_serial_number(): %i\n", error);
        // } else {
        //     printf("serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
        // }

        // Start Measurement

        error = scd4x_start_periodic_measurement();
        // if (error) {
        //     printf("Error executing scd4x_start_periodic_measurement(): %i\n",
        //         error);
        // }

        // printf("Waiting for first measurement... (5 sec)\n");

        // for (;;) {
            // Read Measurement
        sensirion_i2c_hal_sleep_usec(100000);
        bool data_ready_flag = false;
        error = scd4x_get_data_ready_flag(&data_ready_flag);
        if (error) {
            // printf("Error executing scd4x_get_data_ready_flag(): %i\n", error);
            continue;
        }
        if (!data_ready_flag) {
            continue;
        }
        // } //for loop

        uint16_t co2;
        int32_t temperature;
        int32_t humidity;
        error = scd4x_read_measurement(&co2, &temperature, &humidity);
        // if (error) {
        //     printf("Error executing scd4x_read_measurement(): %i\n", error);
        // } else if (co2 == 0) {
        //     printf("Invalid sample detected, skipping.\n");
        // } else {
        //     printf("CO2: %u\n", co2);
        //     printf("Temperature: %d m°C\n", temperature);
        //     printf("Humidity: %d mRH\n", humidity);

        // }
        if ( !error && co2!=0 ){
            data->CO2 = (uint32_t) co2;
            data->temperature = (uint32_t) temperature;
            data->humidity = (uint32_t) humidity;

        }
        xSemaphoreGive(I2C1_Mutex);

        // u32_sensor_data_ready |= bSCD4x_BIT;
        xEventGroupSetBits(
                            xSensorsEventGroup,    
                            bSCD4x_BIT );
        const TickType_t xDelay = portMAX_DELAY / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
        // } //for loop

    }//for loop task
}
// void read_CO_data(void *pvParameters){
//     data->CO=234;
// }
void read_NH3_data_task(void *pvParameters){
    for(;;){
        xSemaphoreTake(I2C1_Mutex, portMAX_DELAY);

        sensor_data_fusion_t *data = pvParameters;
        data->NH3=2324; 
        xSemaphoreGive(I2C1_Mutex);

        // u32_sensor_data_ready |= b_NH3_BIT;
        xEventGroupSetBits(
                            xSensorsEventGroup,    
                            b_NH3_BIT );
        const TickType_t xDelay = portMAX_DELAY / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
    }
}
void tx_to_controller_task(void* pvParameters)
{
    sensor_data_fusion_t *data = pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    for(;;){
      
        //wait for all sensor finnish their tasks
        /* using var */
        // while(u32_sensor_data_ready != ALL_SENSORS_DATE_READY){
        //     vTaskDelay( xDelay );
        // }
        // u32_sensor_data_ready=0;
        /* using event*/
        EventBits_t uxBits;
        //  wait 150s max
        const TickType_t xTicksToWait = 150000 / portTICK_PERIOD_MS;

        /* Wait a maximum of 120s for either bit 0 or bit 4 to be set within
        the event group.  Clear the bits before exiting. */
        uxBits = xEventGroupWaitBits(
                    xSensorsEventGroup,  
                    ALL_SENSORS_DATE_READY, 
                    pdTRUE,       
                    pdFALSE,      
                    xTicksToWait );
        if( ( uxBits & ( ALL_SENSORS_DATE_READY) ) == ALL_SENSORS_DATE_READY )
        {
            /* xEventGroupWaitBits() returned because both bits were set. */
            
            
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
        else{
            //timeout and there were something wrong with sensors..
        }        
        //trigger sleep mode
        // EventBits_t uxBits;
        /* Set bit 0 and bit 4 in xSensorsEventGroup. */
        xEventGroupSetBits(
                                xSensorsEventGroup,    
                                b_GOTO_SLEEP_FOREVER );
        
        const TickType_t xDelay = portMAX_DELAY / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
        
    }
}
 
void re_init_hw(void){
    return;
}

