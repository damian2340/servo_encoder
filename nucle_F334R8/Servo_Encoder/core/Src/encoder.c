/*
 * encoder.c
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#include "encoder.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

volatile encoder_t* encoderH;

uint32_t encoderLastTime;
uint32_t encoderThisTime;

void encoderInit(encoder_t* encoder, uint16_t resolution) {
    encoderH = encoder;
    encoderH->resolution = resolution;
    encoderH->speedBaseTime = 2 * resolution * (htim2.Init.Prescaler + 1);
    encoderH->rev = -1;
    encoderH->offset = 0;

    MX_TIM2_Init();
    MX_TIM1_Init(encoderH->resolution);
}

void encoderStart() {
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

int32_t getEncoderPos() {
    encoderH->userPos = encoderH->sysPos + encoderH->rev * (encoderH->resolution) + encoderH->offset;

    return encoderH->userPos;
}

void setEncoderOffset(int32_t offset) {
    encoderH->offset = offset;
}

void setResolution(int16_t resolution) {
    encoderH->resolution = resolution;
    encoderH->rev = -1;
    encoderH->lastSysPos = 0;
    encoderH->sysPos = 0;
    encoderH->time = 0;
    encoderH->timeA = 0;
    encoderH->timeB = 0;
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetAutoreload(&htim1, (encoderH->resolution)-1 );
    __HAL_TIM_SetCounter(&htim1, 0);
}

void setEncoderPos(int32_t pos) {
    encoderH->rev = pos / (encoderH->resolution);
    if (pos < 0) {
        pos += 0xFFFFFFFF;
    }
    __HAL_TIM_SetCounter(&htim1, (uint32_t)pos%(encoderH->resolution) );
}

uint32_t getEncoderPer() {
    if (encoderH->period > getEncoderBoundedPer()) {
        return encoderH->period;
    }
    return encoderH->bouiundedPeriod;

}

uint32_t getEncoderBoundedPer() {
    uint32_t encoderThisTime;
    if (encoderH->timeB > encoderH->timeA) {
        encoderLastTime = encoderH->timeA;
    } else {
        encoderLastTime = encoderH->timeB;
    }
    encoderThisTime = getEncoderTime();
    if (encoderLastTime < encoderThisTime) {
        encoderThisTime -= encoderLastTime;
    } else {
        encoderThisTime += (0xFFFFFFFF - encoderLastTime);
    }
    encoderH->bouiundedPeriod = encoderThisTime;
    return encoderThisTime;
}

uint32_t getEncoderTime() {
    return __HAL_TIM_GetCounter(&htim2);
}

float getEncoderSpeedHz() {
    encoderH->speed = encoderH->direction
            * (float) ((double) HAL_RCC_GetPCLK1Freq() / ((double) getEncoderPer() * (double) encoderH->speedBaseTime));

    return encoderH->speed;
}

float getEncoderBoundedSpeedHz() {
    return (double) HAL_RCC_GetPCLK1Freq() / ((double) getEncoderBoundedPer() * (double) encoderH->speedBaseTime);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static int32_t step = 0;
    if (htim->Instance == TIM2) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                encoderLastTime = encoderH->timeA;
                encoderH->timeA = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);
                if (encoderLastTime < encoderH->timeA) {
                    encoderH->period = encoderH->timeA - encoderLastTime;
                } else {
                    encoderH->period = encoderH->timeA + (0xFFFFFFFF - encoderLastTime);
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                encoderLastTime = encoderH->timeB;
                encoderH->timeB = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_2);
                if (encoderLastTime < encoderH->timeB) {
                    encoderH->period = encoderH->timeB - encoderLastTime;
                } else {
                    encoderH->period = encoderH->timeB + (0xFFFFFFFF - encoderLastTime);
                }
                break;
            default:
                Error_Handler();
        }
        encoderH->lastSysPos = encoderH->sysPos;
        encoderH->sysPos = __HAL_TIM_GetCounter(&htim1);
        step = encoderH->sysPos - encoderH->lastSysPos;
        if (step == -1 || step == 511) {
            encoderH->direction = -1;
        } else if (step == 1 || step == -511) {
            encoderH->direction = 1;
        }
    } else {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        encoderH->sysPos = __HAL_TIM_GetCounter(&htim1);
        if (encoderH->sysPos == 0) {
            encoderH->rev++;
        } else if (encoderH->sysPos == encoderH->resolution - 1) {
            encoderH->rev--;
        }
    } else if (htim->Instance == TIM2) {
        // TODO: si se cambia encoderTimer a uint64_t se debe agragar o quitar 1 al accarreo en cada desborde del contador del timer 2.
    } else if (htim->Instance == TIM6) {
        HAL_IncTick();
    } else {
        Error_Handler();
    }
}
