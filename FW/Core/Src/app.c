#include "app.h"
//function prototype;
void read_sensirion_data(sensor_data_fusion_t *data);
// void read_CO_data(sensor_data_fusion_t *data);
void read_NH3_data(sensor_data_fusion_t *data);
void re_init_hw(void);
void read_sensor_data(sensor_data_fusion_t *data);
void read_light_sensor(sensor_data_fusion_t *data);
void read_AM2320_hu_temp_data(sensor_data_fusion_t *data);
void read_audio_data(sensor_data_fusion_t *data);
//global vars
sensor_data_fusion_t m_sensor_data_fusion;

void read_light_sensor(sensor_data_fusion_t *data){
    int Vpr=0;
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
      Vpr = HAL_ADC_GetValue(&hadc);
    }
    //  Vpr=0.33*(1+L_i)
    // -->Li = Vpr*3-1 (Lux) (min)
    data->light = Vpr*3 -1;
}


void read_AM2320_hu_temp_data(sensor_data_fusion_t *data){
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
    
    // my_printf("Temperature: %.1fºC\r\n", temperature);
    // my_printf("Humidity: %.1f%%\r\n", humidity);
		/* USER CODE END WHILE */
}

void read_audio_data(sensor_data_fusion_t *data){
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
    //  Vpr=0.33*(1+L_i)
    // -->Li = Vpr*3-1 (Lux) (min)
    data->audio = adc_val;
}

void read_sensirion_data(sensor_data_fusion_t *data) {
    int16_t error = 0;

    sensirion_i2c_hal_init();

    // Clean up potential SCD40 states
    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    scd4x_reinit();

    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
    if (error) {
        printf("Error executing scd4x_get_serial_number(): %i\n", error);
    } else {
        printf("serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    }

    // Start Measurement

    error = scd4x_start_periodic_measurement();
    if (error) {
        printf("Error executing scd4x_start_periodic_measurement(): %i\n",
               error);
    }

    printf("Waiting for first measurement... (5 sec)\n");

    for (;;) {
        // Read Measurement
        // sensirion_i2c_hal_sleep_usec(100000);
        bool data_ready_flag = false;
        error = scd4x_get_data_ready_flag(&data_ready_flag);
        if (error) {
            // printf("Error executing scd4x_get_data_ready_flag(): %i\n", error);
            continue;
        }
        if (!data_ready_flag) {
            continue;
        }
    } //for loop

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
    // } //for loop

    return 0;
}
// void read_CO_data(sensor_data_fusion_t *data){
//     data->CO=234;
// }
void read_NH3_data(sensor_data_fusion_t *data){
    data->NH3=2324;
}
void re_init_hw(void){
    return;
}


void read_sensor_data(sensor_data_fusion_t *data){
    // uint16_t data;
    read_light_sensor(data);
    read_AM2320_hu_temp_data(data);
    read_audio_data(data);
    read_sensirion_data(data);
    // read_CO_data(data);
    read_NH3_data(data);

}
void main_task(void){
    // for(;;){
    read_sensor_data(&m_sensor_data_fusion);
    tx_to_controller(&m_sensor_data_fusion);
        // HAL_PWR_EnterSTANDBYMode();
        // re_init_hw();
    // }
}