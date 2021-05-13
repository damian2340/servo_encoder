/*
 * encoder.h
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#define ENCORER_RESOLUTION 511


typedef struct encoderTD{
    //configuración
    int16_t resolution;
    int32_t offset;
    uint32_t speedBaseTime;

    //posición
    int32_t rev;
    int32_t sysPos;
    int32_t lastSysPos;
    int32_t userPos;

    //tiempo y velocidad
    uint32_t time;
    uint32_t timeA;
    uint32_t timeB;
    uint32_t period;
    uint32_t bouiundedPeriod;
    float speed;
    float direction;
}encoder_t;

void encoderInit(encoder_t* encoder , uint16_t resolution );
void encoderStart();
void setResolution(int16_t resolution);
int32_t getEncoderPos();
void setEncoderPos( int32_t pos );
void setEncoderOffset(int32_t offset);
uint32_t getEncoderPer();
uint32_t getEncoderBoundedPer();
uint32_t getEncoderTime();
float getEncoderSpeedHz();
float getEncoderBoundedSpeedHz();
float getEncoderdir();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif /* SRC_ENCODER_H_ */
