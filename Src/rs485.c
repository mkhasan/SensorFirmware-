
#include "common.h"
#include "rs485.h"

tUart data;

extern int countX;

extern int serialData;
extern UART_HandleTypeDef huart1;

uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??

int ucReceive_Event = 0;

static uint8_t data_element;

static int CheckID(uint8_t *id);

static int IsValid(uint8_t value);

static int CheckCmd();



void rs485_Init() {
  data.Flag = 0;
  data.rx_point_head = data.rx_point_tail = data.tx_point_head = data.tx_point_tail = 0;
  data.cmdIndex = 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    
    //AL_UART_Transmit(&huart1, (uint8_t *)data.TxBuf, 10,0xFFFF);
    //serialData ++;
    
    countX = 1;
    
    int i;
    for(i=0; i<RX1BUFFERSIZE;i++)
    {
      data.RxBuf[data.rx_point_head++] = ucpRx1Buffer[i];	//?? ???? ??? ??
      data.rx_point_head %= RBUF_SIZE;                 
    }  
    int ret;
    
    /*
    if ((ret=RequestData()) != HAL_OK) 
    {
      Error_Handler();
    }
*/
  }
}

int RequestData() {
  return HAL_UART_Receive_IT(&huart1, (uint8_t *)ucpRx1Buffer, RX1BUFFERSIZE);
    
}

void ProcessCmd() {

  if(data.rx_point_tail == data.rx_point_head)
    return;
    
  data_element = data.RxBuf[data.rx_point_tail++];
  data.rx_point_tail %= RBUF_SIZE;
  
  if(data.Flag == 0 && data_element != STX)
    return;
    
  if(data.Flag==1 && data_element == ETX) {  
       
    if(data.cmdIndex == CMD_LEN)
      ucReceive_Event = 1;
    
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
    

  
  
  if(ucReceive_Event == 1) {
    
    if(CheckCmd() == 1)
      SendValue();
    ucReceive_Event = 0;
  }
}    
    


int CheckID(uint8_t * id) {
  return 1;
}


 

int IsValid(uint8_t value) {
  if(value == ' ' || (value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9' ))
    return 1;
  
  return 0;
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
  for(i=0; i<26; i++)
    data.TxBuf[i] = 'X';
  
  data.TxBuf[0] = 'V';
  data.TxBuf[1] = 'A';
  data.TxBuf[2] = 'L';
  data.TxBuf[3] = 'U';
  data.TxBuf[4] = 'E';
  
  if(HAL_UART_Transmit(&huart1, data.TxBuf, 20, 0xFFFF)!= HAL_OK) {
    serialData --;
  }
}