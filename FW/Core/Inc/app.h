#ifndef __APP_H
#define __APP_H
// #include "i2c.h"
#include  "am2320.h" 
#include "sensor_to_ctrler.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
// #define   AM2320_SLAVE_ADDR=0xB8;

//ID of inside sensor from 0-0x99
#define MAX_INSIDE_SENSOR_ID 0x99
typedef struct {
  unsigned int sensorID;
  unsigned int temperature;
  unsigned int humidity;
  unsigned int AM2320_temperature;
  unsigned int AM2320_humidity;
  unsigned int audio;
  unsigned int light;
  unsigned int accleration;
  unsigned int CO2;
  unsigned int NH3;
} sensor_data_fusion_t;

// typedef struct Point Point;
typedef struct 
{ /* Mail object structure */
  bool bAM2320; 
  bool bSCD4x; 
  bool b_light; 
  bool b_audio; 
  bool b_NH3; 
} sensor_data_ready_t;
#define bAM2320_BIT  (1<<0)
#define bSCD4x_BIT  (1<<1)
#define b_light_BIT  (1<<2)
#define b_audio_BIT  (1<<3)
#define b_NH3_BIT  (1<<4)
#define b_GOTO_SLEEP_FOREVER (1<<7)
#define ALL_SENSORS_DATE_READY    (bAM2320_BIT|bSCD4x_BIT|b_light_BIT| \
        b_audio_BIT|b_NH3_BIT)

// void main_task(void);
void main_task( void * pvParameters );
void read_sensirion_data_task(void *pvParameters);
// void read_CO_data(void *pvParameters);
void read_NH3_data_task(void *pvParameters);
void re_init_hw(void);
void read_sensor_data(void *pvParameters);
void read_light_sensor_task(void *pvParameters);
void read_AM2320_hu_temp_data(void *pvParameters);
void read_audio_data_task(void *pvParameters);
void power_saving_task(void *pvParameters);
void tx_to_controller_task(void* pvParameters);



#endif