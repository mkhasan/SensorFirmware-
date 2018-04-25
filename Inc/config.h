#ifndef _CONFIG_H
#define _CONFIG_H

#define SENSOR1_PORT GPIOA
#define SENSOR2_PORT SENSOR1_PORT

#define SENSOR_PORT SENSOR1_PORT

#define SENSOR_PIN(X) GPIO_PIN_ ## X

#define SENSOR_CHANNEL(X) ADC_CHANNEL_ ## X 


#define SENSOR1_PIN GPIO_PIN_5
#define SENSOR2_PIN GPIO_PIN_7


#define SENSOR1_CHANNEL ADC_CHANNEL_5
#define SENSOR2_CHANNEL ADC_CHANNEL_7

#define SENSOR1_PIN SENSOR_PIN(5)
#endif