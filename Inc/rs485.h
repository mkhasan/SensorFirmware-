#ifndef RS485_H_
#define RS485_H_

#include "stm32f1xx_hal.h"

#define RBUF_SIZE 1024
#define TBUF_SIZE RBUF_SIZE

#define RX1BUFFERSIZE 16

#define STX 'X'
#define ETX 'Y'

#define DATA_KEY 0xA5

#define ADDR_PORT GPIOB

#define ADDR0_PIN GPIO_PIN_15
#define ADDR1_PIN GPIO_PIN_14
#define ADDR2_PIN GPIO_PIN_13
#define ADDR3_PIN GPIO_PIN_12


typedef struct{
    uint8_t Flag; 

    
    uint16_t rx_point_tail;   
    uint16_t rx_point_head;   
    uint16_t tx_point_tail;
    uint16_t tx_point_head;

    uint8_t RxBuf[RBUF_SIZE]; 
    uint8_t TxBuf[TBUF_SIZE];
    
    uint8_t CurrentCmd[RX1BUFFERSIZE];          // must not be greater than RX1BUFFERSIZE
    int cmdIndex;
}tUart;

void rs485_Init();
void RequestRecv();
void ProcessInput();
void SendData();

void GetAddr();

#endif 