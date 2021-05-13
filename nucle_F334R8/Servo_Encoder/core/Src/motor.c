/*
 * motor.c
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#include "motor.h"

extern HRTIM_HandleTypeDef hhrtim1;

motor_t* motorH;

void motorInit(motor_t* motor, float maxVoltage) {
    motorH = motor;
    motor->maxVoltage = maxVoltage;
    pinMotorInit();
    motorDisable();
    MX_HRTIM1_Init();
}

void pinMotorInit() {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /*Configure GPIO pin Output Level */
    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void motorEnable() {
    if (HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A) == HAL_OK
            && HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) == HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    }
}
void motorDisable() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

void setMaxVoltage(float maxVoltage) {
    motorH->maxVoltage = maxVoltage;
}

float getMaxVoltage(void){
    return motorH->maxVoltage;
}

void setVoltage(float voltage) {
    if(voltage > motorH->maxVoltage)
        motorH->voltage = motorH->maxVoltage;
    else if(voltage < -1 * motorH->maxVoltage )
        motorH->voltage = -1 * motorH->maxVoltage;
    else
        motorH->voltage = voltage;

    setPWM(motorH->voltage / motorH->maxVoltage);
}

float getVoltage(){
    return motorH->voltage;
}

void setPWM(float value) {
    if (value > 1) {
        value = 1;
    } else if (value < -1) {
        value = -1;
    }
    uint16_t halfPeriod = ((__HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A)) + 1) / 2;
    uint16_t pwm = halfPeriod + (halfPeriod - 48) * value;
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, pwm);
}
