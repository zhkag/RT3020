

#ifndef SENSOR_RT_RT3020_H__
#define SENSOR_RT_RT3020_H__

#include "sensor.h"
#include "rt3020.h"

#define RT3020_ADDR_DEFAULT (RT3020_I2C_ADDRESS_SDO_LOW)

int rt_hw_rt3020_init(const char *name, struct rt_sensor_config *cfg);

#endif
