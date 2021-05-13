/*
 * serialCom.c
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#include "serialCom.h"
#include "encoder.h"
#include "motor.h"

volatile bufferOut_t bufferOutputOdd;
volatile bufferOut_t bufferOutputEven;
volatile bufferIn_t bufferInput;

uint8_t commandReady = 0;

extern UART_HandleTypeDef huart2;

int32_t tempPos;
float tempSpeed;

void serialComInit() {
    MX_DMA_Init();
    MX_USART2_UART_Init();
}

HAL_UART_StateTypeDef startRecive() {
    if (huart2.RxState == HAL_UART_STATE_READY) {
        flushBuffer();
        bufferInput.state = BufferInBusyState;
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart2, (uint8_t*) bufferInput.data, (uint16_t) (BUFFER_IN_SIZE));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
    return huart2.RxState;
}

void flushBuffer() {
    memset((char*) (bufferInput.data), 0, BUFFER_IN_SIZE);
    bufferInput.index = 0;
    bufferInput.state = BufferInIdleState;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART2) {
        if (bufferInput.state == BufferInBusyState) {
            commandReady = 1;
        }
    }
}

void uartSetReady() {
    UART_HandleTypeDef *huart = &huart2;
    // Whether dma transfer is incomplete
    if (huart->hdmarx->Instance->CNDTR < (BUFFER_IN_SIZE - bufferInput.index)) {
        //HAL_UART_DMAStop(&huart2);

        /* Stop UART DMA Rx request if ongoing */
        if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))) {
            CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

            /* Abort the UART DMA Rx channel */
            if (huart->hdmarx != NULL) {
                HAL_DMA_Abort(huart->hdmarx);
            }

            /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
            CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
            CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

            /* At end of Rx process, restore huart->RxState to Ready */
            huart->RxState = HAL_UART_STATE_READY;
        }

    }

}

/******************************************************************************************
 *                                  TRANSMISIÓN
 ******************************************************************************************/
void prepareDataToSend() {
    tempPos = getEncoderPos();
    tempSpeed = getEncoderSpeedHz();
    if (bufferOutputOdd.state == BufferOutIdleState) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        bufferOutputOdd.state = BufferOutBusyState;
        sprintf((char*) bufferOutputOdd.data, "%09i,%+08.6e\r\n", (int) tempPos, (float) tempSpeed);
        if (bufferOutputEven.state == BufferOutReady4SendState) {
            bufferOutputEven.state = BufferOutIdleState;
        }
        bufferOutputOdd.state = BufferOutReady4SendState;
    } else if (bufferOutputEven.state == BufferOutIdleState) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        bufferOutputEven.state = BufferOutBusyState;
        sprintf((char*) bufferOutputEven.data, "%09i,%+08.6e\r\n", (int) tempPos, (float) tempSpeed);
        if (bufferOutputOdd.state == BufferOutReady4SendState) {
            bufferOutputOdd.state = BufferOutIdleState;
        }
        bufferOutputEven.state = BufferOutReady4SendState;
    }
}

void serialTransmit() {
    static uint8_t noData[10] = { "no data \r\n" };
    if (huart2.gState == HAL_UART_STATE_READY) {
        if (bufferOutputOdd.state == BufferOutReady4SendState) {
            bufferOutputOdd.state = BufferOutSendingState;
            HAL_UART_Transmit_DMA(&huart2, (uint8_t *) bufferOutputOdd.data, 25);
        } else if (bufferOutputEven.state == BufferOutReady4SendState) {
            bufferOutputEven.state = BufferOutSendingState;
            HAL_UART_Transmit_DMA(&huart2, (uint8_t *) bufferOutputEven.data, 25);
        } else {
            HAL_UART_Transmit_DMA(&huart2, noData, 24);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (bufferOutputOdd.state == BufferOutSendingState) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            bufferOutputOdd.state = BufferOutIdleState;
        } else if (bufferOutputEven.state == BufferOutSendingState) {
            bufferOutputEven.state = BufferOutIdleState;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}
