/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include "ultrasonic.h"
#include "delay_us.h"
#include <stdio.h>      // printf 사용 위해 포함

/* Private define ------------------------------------------------------------*/
#define SAFE_DISTANCE 20   // cm

/* Private variables ---------------------------------------------------------*/
uint32_t front_distance;
uint32_t left_distance;
uint32_t right_distance;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void AutonomousDrive(void);

/* USER CODE BEGIN 0 */
// 디버깅용 _write() 제거 또는 필요시 SEGGER RTT 등으로 대체 가능
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_TIM10_Init();

    HAL_TIM_Base_Start(&htim10);              // delay_us용 타이머
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // 초음파 캡처 인터럽트

    while (1)
    {
        AutonomousDrive();
    }
}

/* AutonomousDrive 함수 정의 - main 밖으로 이동 */
void AutonomousDrive(void)
{
    // 거리 측정
    HCSR04_TRIGGER_FRONT();
    HAL_Delay(50);
    front_distance = distance;

    HCSR04_TRIGGER_LEFT();
    HAL_Delay(50);
    left_distance = distance;

    HCSR04_TRIGGER_RIGHT();
    HAL_Delay(50);
    right_distance = distance;

    printf("Front:%ld cm, Left:%ld cm, Right:%ld cm\r\n",
           front_distance, left_distance, right_distance);

    if(front_distance < SAFE_DISTANCE)
    {
        Car_Stop();
        HAL_Delay(200);

        Car_Backward();
        HAL_Delay(300);

        // 좌우 회피 결정
        if(left_distance > right_distance)
        {
            Car_Left();
        }
        else
        {
            Car_Right();
        }
        HAL_Delay(400);
    }
    else
    {
        Car_Forward();
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}
