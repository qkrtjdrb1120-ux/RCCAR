/*
 * ultrasonic.h
 *
 *  Created on: Nov 3, 2025
 *      Author: user9
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"
#include "tim.h"
#include "delay_us.h"


// 초음파 센서 관련 매크로
#define TRIG_PORT GPIOA
#define TRIG_PIN  GPIO_PIN_5

// 외부 변수 선언
extern uint16_t IC_Value1;
extern uint16_t IC_Value2;
extern uint16_t echoTime;
extern uint8_t captureFlag;
extern uint8_t distance;

// 함수 선언
void HCSR04_TRIGGER(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* __ULTRASONIC_H */


#endif /* INC_ULTRASONIC_H_ */
