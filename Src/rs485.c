

#include "rs485.h"
#include "dwt_stm32_delay.h"
#include "config.h"
#include "crc.h"

tUart data;
uint32_t addrData;
uint8_t addrCrc;

extern int readCallback;
extern int readyToReceive;

extern int dataReady;
extern int sendData;
extern UART_HandleTypeDef huart1;
extern int sentBufferEmpty;
extern int sendEvent;
extern int writeErrorCount;
extern int lastByte;


uint8_t cmaLen = RX1BUFFERSIZE;

uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??
uint8_t temp = 0;

extern int ucReceive_Event;
extern uint32_t g_ADCValue;
extern int reqReceived;

extern uint32_t g_ADCValueDMA[2];;//uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];    
int turn = 0;

extern int ret;

static int idLen=0;

GPIO_PinState state;

int addr[ADDR_PIN_COUNT];

uint32_t myAddr = 1;

uint8_t cmdStr[266];

uint8_t cmdStrPrefix[256] = "GET VALUE ";

uint8_t cmdLen = RX1BUFFERSIZE;

extern uint8_t myChar;

static uint8_t data_element;

static int CheckID(uint8_t *id);

static int IsValid(uint8_t value);

static int CheckCmd();

void UART_RxAgain(UART_HandleTypeDef *huart);

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
  strcpy(cmdStr, cmdStrPrefix);
  strcat(cmdStr, "NNN");
  cmdLen = strlen(cmdStr);
  
  
  idLen = strlen(cmdStr) - strlen(cmdStrPrefix);
}

void HAL_UART_RxCpltCallback1(UART_HandleTypeDef *huart) {
  
 if(huart == &huart1)
  {
    
    readCallback++;
    
    data.RxBuf[data.rx_point_head++] = myChar;
   
 
  }  
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    
    readCallback++;
    
    int i;
    for(i=0; i<RX1BUFFERSIZE;i++)
    {
      data.RxBuf[data.rx_point_head++] = ucpRx1Buffer[i];	//?? ???? ??? ??
      data.rx_point_head %= RBUF_SIZE;         

      temp = ucpRx1Buffer[0];
      
      //ucpRx1Buffer[i] = 0;
    }  
    
     while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
       ; 
    ucReceive_Event = 1;
    reqReceived = 1;
    readyToReceive = 0;
    
 
  }
}

*/

extern int transferErrorCount;
extern int writeErrorCount;
extern int readErrorCount;
extern uint32_t errCode[256];

void  RequestRecv() {
  
  
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    
    //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
      //;
    
    
  
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    
    //if((ret = HAL_UART_Receive_IT(&huart1, (uint8_t *)ucpRx1Buffer, RX1BUFFERSIZE)) != HAL_OK) 
    
    uint8_t localData;
    int recvRet;
    if((recvRet=HAL_UART_Receive(&huart1,&localData,1, 0xFF)) == HAL_OK) {
      data.RxBuf[data.rx_point_head++] = localData;
      data.rx_point_head %= RBUF_SIZE;
    }
    else {
      errCode[transferErrorCount%256] = recvRet;
      transferErrorCount ++;
      readErrorCount ++;
    }
      
       
    //UART_RxAgain(&huart1);
    
  
  
  
    
}

void ProcessInput() {

  
  if(data.rx_point_tail == data.rx_point_head)
    return;
    
  data_element = data.RxBuf[data.rx_point_tail++];
  data.rx_point_tail %= RBUF_SIZE;
  
  if(data.Flag == 0 && data_element != STX)
    return;
    
  if(data.Flag==1 && ((data_element == ETX) || (IsValid(data_element) && data.cmdIndex >= cmdLen))) {  
       
    if(data.cmdIndex == cmdLen)
      requestOkay = 1;
    
    data.Flag = 0;
    
  }
  
  else if(data.Flag==1) { 
    
    if(IsValid(data_element) && data.cmdIndex < cmdLen) {
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
    
    if(CheckCmd() == 1) {
      dataReady = 1;
      requestOkay = 0;
    }
    
  }  
 

}    
    


int CheckID(uint8_t * idStr) {          // 1 means valid
  
  
  
uint32_t id = 0;
  
  /*
  uint32_t factor = 1;
  for(int i=0; i<idLen-1; i++)
    factor *= 10;
    
  
  for(i=0; i<idLen; i++) {
    if(!(idStr[i]>='0' && idStr[i] <= '9'))
      return 0;
    id += (idStr[i] - '0')*factor;
    factor /= 10;     
           
  }

  */
  
  
  if(idStr[2] != '0')
    return 0;
  addrData = idStr[0];
  
  addrCrc = crcCalc(addrData, 8, POLYNOM4);
  if (addrCrc != idStr[1])
    return 0;
  
  return (idStr[0] == myAddr);
  
  

  
  
  
   
}


 

int IsValid(uint8_t value) {           // 1 means valid 
  
  
  if(data.cmdIndex == 10)
    data.cmdIndex = 10;
  if(cmdStr[data.cmdIndex] == 'N')  
    return  1;
    
  
  
  if(!(value == ' ' || (value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9' ))) 
    return 0;
  
  if(value != cmdStr[data.cmdIndex])
    return 0;
  
  return 1;
    
    

}

int CheckCmd() {
  
  int len = strlen(cmdStrPrefix);
  
  int i=0;
  for(i=0; i<len; i++)
    if(data.CurrentCmd[i] != cmdStrPrefix[i])
      return 0;
  
  return CheckID(&data.CurrentCmd[len]);
  
}

void SendData() {
  
  
  
  int i;
  /*
  for(i=0; i<10; i++)
    data.TxBuf[i] = '0'+i;
  
  for(i=0; i<20; i++) 
    data.TxBuf[10+i] = 'A'+i;
  */

  for(i=0; i<10; i++)
    data.TxBuf[i] = '0';
  
  data.TxBuf[0] = DATA_KEY;
  data.TxBuf[1] = ' ';
  
  
  if(g_ADCValue < 4096) {
    uint32_t val = g_ADCValue;
      
    /*
    for (i=0; i<4; i++) {
      data.TxBuf[3-i] = '0'+val%10;
      val = val/10;
        
    }
    */
    
    if(turn == 0)
      g_ADCValue = g_ADCValueDMA[0];
    else 
      g_ADCValue = g_ADCValueDMA[1];
    
    turn = 1-turn;
    
    memcpy(&data.TxBuf[2], sendData, sizeof(sendData));
    
    data.TxBuf[2] = sendData & 0xFF;
    data.TxBuf[3] = (sendData & 0xFF00) >> 8;
    data.TxBuf[4] = (sendData & 0xFF0000) >> 16;
    data.TxBuf[5] = myAddr;//(sendData & 0xFF000000) >> 24;
    
    int index = 2+sizeof(sendData);
    
    data.TxBuf[index] = ((g_ADCValueDMA[0] & 0xff00) >> 8);
    data.TxBuf[index+1] = (g_ADCValueDMA[0] & 0xff);
    
    index += 2;
    data.TxBuf[index] = ((g_ADCValueDMA[1] & 0xff00) >> 8);
    data.TxBuf[index+1] = (g_ADCValueDMA[1] & 0xff);

    
    
    
    //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
      //;
    

    
    
    //__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
    
    //DWT_Delay_us(100000);
    
   // 
    
   
    sentBufferEmpty = 0;
    sendData ++;
    lastByte = -1*WRITE_DATA_LEN;

    //__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    
    if(HAL_UART_Transmit_IT(&huart1, data.TxBuf, 10)!= HAL_OK) {
      writeErrorCount ++;
      sentBufferEmpty = 1;
      lastByte = 0;
      sendData --;
      
      
    }
 
    
    
    //while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
       //; 
    

  
  }

}

void GetAddr() {
  
  //state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  
  addr[0] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_0) == GPIO_PIN_SET);
  addr[1] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_1) == GPIO_PIN_SET);
  addr[2] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_2) == GPIO_PIN_SET);
  addr[3] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_3) == GPIO_PIN_SET);
  /*
  addr[4] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_4) == GPIO_PIN_SET);
  addr[5] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_5) == GPIO_PIN_SET);
  addr[6] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_6) == GPIO_PIN_SET);
  addr[7] = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_7) == GPIO_PIN_SET);
*/
  
  int i;
  
  for(i=0, myAddr=0; i<ADDR_PIN_COUNT; i++)
    myAddr |= (addr[i] << i);
   
  
}

void UART_RxAgain(UART_HandleTypeDef *huart) {
  // use the code from HAL_UART_Receive_IT() to repost interrupt
  /*
  if((ret = HAL_UART_Receive_IT(&huart1, (uint8_t *)&myChar, 1)) != HAL_OK) 
    Error_Handler();
  else {
    reqReceived = 0;
  }
*/
  
 /* Check that a Rx process is not already ongoing */
  

    /* Process Locked */
    //__HAL_LOCK(huart);
  
    reqReceived = 0;

    huart->pRxBuffPtr = &myChar;
    huart->RxXferSize = 1;
    huart->RxXferCount = 1;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    
    if(huart->RxState == HAL_UART_STATE_BUSY_TX)  {
      huart->RxState = HAL_UART_STATE_BUSY_TX_RX;
    }
    else {
      huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
    
    //huart->RxState = HAL_UART_STATE_BUSY_RX;
    
    /* Process Unlocked */
    //__HAL_UNLOCK(huart);

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

    return ;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  /* consume the received character */

  if(huart == &huart1)
  {
    
    readCallback++;
    
    data.RxBuf[data.rx_point_head++] = myChar;
    data.rx_point_head %= RBUF_SIZE;         
  
    reqReceived = 1;
    
  }  
  
}

