#ifndef RS485_H_
#define RS485_H_

#include "stm32f1xx_hal.h"

#define RBUF_SIZE 1024
#define TBUF_SIZE RBUF_SIZE

#define RX1BUFFERSIZE 16

#define STX 0x02
#define ETX 0x03

#define CMD_LEN 14

typedef struct{
    uint8_t Flag; 

    
    uint16_t rx_point_tail;   
    uint16_t rx_point_head;   
    uint16_t tx_point_tail;
    uint16_t tx_point_head;

    uint8_t RxBuf[RBUF_SIZE]; 
    uint8_t TxBuf[TBUF_SIZE];
    
    uint8_t CurrentCmd[CMD_LEN];
    int cmdIndex;
}tUart;

void rs485_Init();
int RequestData();
void ProcessCmd();
void SendValue();

#endif 