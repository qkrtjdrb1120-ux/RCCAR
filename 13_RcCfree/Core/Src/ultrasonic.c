/*
 * ultrasonic.c
 *
 *  Created on: Nov 3, 2025
 *      Author: user9
 */


#include "ultrasonic.h"
#include "stdio.h"

// 전역 변수 정의
uint16_t IC_Value1 = 0;
uint16_t IC_Value2 = 0;
uint16_t echoTime = 0;
uint8_t captureFlag = 0;
uint8_t distance = 0;

/**
  * @brief 초음파 센서 트리거 신호 발생
  */
void HCSR04_TRIGGER(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}

/**
  * @brief 입력 캡처 콜백 (초음파 에코 측정)
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (captureFlag == 0)
        {
            IC_Value1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
            captureFlag = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (captureFlag == 1)
        {
            IC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            if (IC_Value2 > IC_Value1)
                echoTime = IC_Value2 - IC_Value1;
            else
                echoTime = (0xFFFF - IC_Value1) + IC_Value2;

            distance = echoTime / 58; // cm 단위로 변환
            captureFlag = 0;
             __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);  // ❌ 이 줄 삭제
        }
    }
}
