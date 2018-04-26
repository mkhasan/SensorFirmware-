#ifndef _CONFIG_H
#define _CONFIG_H

#define SENSOR1_PORT GPIOA
#define SENSOR2_PORT SENSOR1_PORT

#define SENSOR_PORT SENSOR1_PORT



#define SENSOR_PIN(X) GPIO_PIN_ ## X

#define SENSOR_CHANNEL(X) ADC_CHANNEL_ ## X 


#define SENSOR1_PIN GPIO_PIN_4
#define SENSOR2_PIN GPIO_PIN_6

#define SENSOR_PIN_SET ( SENSOR1_PIN | SENSOR2_PIN)



#define SENSOR1_CHANNEL ADC_CHANNEL_4
#define SENSOR2_CHANNEL ADC_CHANNEL_6

#define SENSOR1_SAMPLING_TIME ADC_SAMPLETIME_1CYCLE_5
#define SENSOR2_SAMPLING_TIME SENSOR1_SAMPLING_TIME

#define WRITE_DATA_LEN 2

#define ADC_BUFFER_LENGTH 256
#endif