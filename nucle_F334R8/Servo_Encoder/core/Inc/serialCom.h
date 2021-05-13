/*
 * serialCom.h
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#ifndef INC_SERIALCOM_H_
#define INC_SERIALCOM_H_

#include "dma.h"
#include "usart.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"


#define BUFFER_IN_SIZE 16U
#define BUFFER_OUT_SIZE 32U


typedef enum BuferOutStateTD{
    BufferOutIdleState         =  0x00,
    BufferOutBusyState         =  0x01,
    BufferOutReady4SendState   =  0x02,
    BufferOutSendingState      =  0x04,
    BufferOutErrorState        = -0x01
}bufferOutState_t;

typedef struct bufferTD{
    uint8_t data[BUFFER_OUT_SIZE];
    bufferOutState_t state;
}bufferOut_t;

typedef enum BuferinStateTD{
    BufferInIdleState =  0x00,
    BufferInBusyState =  0x01,
    BufferInReadyState     =  0x02,
    BufferInErrorState= -0x01
}bufferInState_t;

typedef struct bufferInTD{
    uint8_t data[BUFFER_IN_SIZE];
    bufferInState_t state;
    size_t index;
}bufferIn_t;

void serialComInit();
void serialTransmit();
HAL_UART_StateTypeDef startRecive();
void flushBuffer();
void prepareDataToSend();
void uartSetReady();
#endif /* INC_SERIALCOM_H_ */
