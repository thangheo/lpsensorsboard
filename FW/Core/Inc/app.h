#ifndef __APP_H
#define __APP_H
// #include "i2c.h"
#include  "am2320.h" 
#include "sensor_to_ctrler.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
// #define   AM2320_SLAVE_ADDR=0xB8;

typedef struct {
  unsigned int temperature;
  unsigned int humidity;
  unsigned int AM2320_temperature;
  unsigned int AM2320_humidity;
  unsigned int audio;
  unsigned int light;
  unsigned int CO;
  unsigned int CO2;
  unsigned int NH3;
} sensor_data_fusion_t;
// typedef struct Point Point;



void main_task(void);







#endif