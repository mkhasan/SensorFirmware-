
#include "common.h"
#include "rs485.h"
#include "dwt_stm32_delay.h"

tUart data;

extern int readCallback;
extern int readyToReceive;

extern int dataReady;
extern int sendData;
extern UART_HandleTypeDef huart1;
extern int sentBufferEmpty;
extern int sendEvent;
extern int writeErrorCount;



uint8_t cmaLen = RX1BUFFERSIZE;

uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??
uint8_t temp = 0;

extern int ucReceive_Event;
extern uint32_t g_ADCValue;
extern int reqReceived;

int ret = 0;

static int idLen=0;

GPIO_PinState state;

int addr0, addr1, addr2, addr3;

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

void  RequestRecv() {
  
  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    
    //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
      //;
    
    
  
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    
    //if((ret = HAL_UART_Receive_IT(&huart1, (uint8_t *)ucpRx1Buffer, RX1BUFFERSIZE)) != HAL_OK) 
    
    UART_RxAgain(&huart1);
    readyToReceive = 1;
  
  
  
    
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
    


int CheckID(uint8_t * idStr) {
  
  
  /*
  int i;
  uint32_t id = 0;
  uint32_t factor = 1;
  for(int i=0; i<idLen-1; i++)
    factor *= 10;
    
  
  for(i=0; i<idLen; i++) {
    if(!(idStr[i]>='0' && idStr[i] <= '9'))
      return 0;
    id += (idStr[i] - '0')*factor;
    factor /= 10;     
           
  }

  
  
   
  return (myAddr == id);

  */
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
    
    
    data.TxBuf[2] = ((g_ADCValue & 0xff00) >> 8);
    data.TxBuf[3] = (g_ADCValue & 0xff);
    
    
    
    //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
      //;
    

    
    
    //__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    
    //DWT_Delay_us(100000);
    
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    
    if(HAL_UART_Transmit_IT(&huart1, data.TxBuf, 10)!= HAL_OK) {
      writeErrorCount ++;
      
    }
    else {
      sentBufferEmpty = 0;
      sendData ++;
      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
 

     //while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
       //; 
    
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  
  }

}

void GetAddr() {
  
  //state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
  
  addr0 = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR0_PIN) == GPIO_PIN_SET);
  addr1 = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR1_PIN) == GPIO_PIN_SET);
  addr2 = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR2_PIN) == GPIO_PIN_SET);
  addr3 = (HAL_GPIO_ReadPin(ADDR_PORT, ADDR3_PIN) == GPIO_PIN_SET);
  
  myAddr = addr3 << 3 | addr2 << 2 | addr1 << 1 | addr0;
  
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

