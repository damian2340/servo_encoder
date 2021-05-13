/*
 * motro.h
 *
 *  Created on: 3 may. 2021
 *      Author: damian
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "gpio.h"
#include "hrtim.h"


typedef struct MotorTD{
        float maxVoltage;
        float voltage;
}motor_t;


void motorInit(motor_t* motor, float maxVoltage);
void pinMotorInit();
void motorEnable();
void motorDisable();
void setMaxVoltage(float maxVoltage);
float getMaxVoltage(void);
void setVoltage(float voltage);
float getVoltage(void);
void setPWM(float value);

#endif /* INC_MOTOR_H_ */
