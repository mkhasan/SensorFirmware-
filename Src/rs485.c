
#include "common.h"
#include "rs485.h"

tUart data;

extern int countX;
extern int readyToReceive;

extern int sendData;
extern UART_HandleTypeDef huart1;



uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??
uint8_t temp = 0;

extern int ucReceive_Event;
extern uint32_t g_ADCValue;

int ret = 0;

uint8_t cmdStr[CMD_LEN+1] = "GET VALUE NNNN";

static uint8_t data_element;

static int CheckID(uint8_t *id);

static int IsValid(uint8_t value);

static int CheckCmd();

int requestOkay = 0;

void rs485_Init() {
  int i;
  data.Flag = 0;
  data.rx_point_head = data.rx_point_tail = data.tx_point_head = data.tx_point_tail = 0;

  for(i=0; i< RBUF_SIZE; i++) {
    data.RxBuf[i] = 0;
    data.TxBuf[i] = 0;
  }
  
  for(i=0; i<RX1BUFFERSIZE; i++)
    ucpRx1Buffer[i] = 'z';
    
  data.cmdIndex = 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    
    //AL_UART_Transmit(&huart1, (uint8_t *)data.TxBuf, 10,0xFFFF);
    //serialData ++;
    
    countX++;
    
    int i;
    for(i=0; i<RX1BUFFERSIZE;i++)
    {
      data.RxBuf[data.rx_point_head++] = ucpRx1Buffer[i];	//?? ???? ??? ??
      data.rx_point_head %= RBUF_SIZE;         
//      if(data.rx_point_head == )
      temp = ucpRx1Buffer[0];
      if((temp < 'A' || temp > 'Z') && countX > 0)
        countX++; 
    }  
    
    ucReceive_Event = 1;
    readyToReceive = 0;
    
   
    
    
    
    
  }
}

void    RequestData() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  if((ret = HAL_UART_Receive_IT(&huart1, (uint8_t *)ucpRx1Buffer, RX1BUFFERSIZE)) != HAL_OK) 
    Error_Handler();
  else
    readyToReceive = 1;
  
  
    
}

void ProcessInput() {

  
  if(data.rx_point_tail == data.rx_point_head)
    return;
    
  data_element = data.RxBuf[data.rx_point_tail++];
  data.rx_point_tail %= RBUF_SIZE;
  
  if(data.Flag == 0 && data_element != STX)
    return;
    
  if(data.Flag==1 && ((data_element == ETX) || (IsValid(data_element) && data.cmdIndex >= CMD_LEN))) {  
       
    if(data.cmdIndex == CMD_LEN)
      requestOkay = 1;
    
    data.Flag = 0;
    
  }
  
  else if(data.Flag==1) { 
    
    if(IsValid(data_element) && data.cmdIndex < CMD_LEN) {
      data.Flag = 1;
      data.CurrentCmd[data.cmdIndex++] = data_element;
    }
    else
      data.Flag = 0;
  }
  
  else if(data_element == STX) {  //1st Start flag
    data.Flag = 1;
    data.cmdIndex = 0;
  }

  if(requestOkay == 1) {
    
    if(CheckCmd() == 1)
      SendValue();
    requestOkay = 0;
  }  
 

}    
    


int CheckID(uint8_t * id) {
  return 1;
}


 

int IsValid(uint8_t value) {
  
  
  if(data.cmdIndex == 10)
    data.cmdIndex = 10;
  if(cmdStr[data.cmdIndex] == 'N')  
    return  (value >= '0' && value <= '9');
    
  
  
  if(!(value == ' ' || (value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9' ))) 
    return 1;
  
  if(value != cmdStr[data.cmdIndex])
    return 0;
  
  return 1;
    
    

}

int CheckCmd() {
  char cmd[] = "GET VALUE ";
  int len = sizeof(cmd)-1;
  
  int i=0;
  for(i=0; i<len; i++)
    if(data.CurrentCmd[i] != cmd[i])
      return 0;
  
  return CheckID(&data.CurrentCmd[len]);
  
}

void SendValue() {
  
  
  
  int i;
  /*
  for(i=0; i<10; i++)
    data.TxBuf[i] = '0'+i;
  
  for(i=0; i<20; i++) 
    data.TxBuf[10+i] = 'A'+i;
  */

  for(i=0; i<10; i++)
    data.TxBuf[i] = '0';
  
  if(g_ADCValue < 4096) {
    uint32_t val = g_ADCValue;
        
    for (i=0; i<4; i++) {
      data.TxBuf[3-i] = '0'+val%10;
      val = val/10;
        
    }
    
  }
 

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  
  if(HAL_UART_Transmit(&huart1, data.TxBuf, 10, 0x10)!= HAL_OK) {
    ret -= 100;
  }
  else

    sendData ++;


}