/*
 * ultrasonic_task.c
 *
 *  Created on: Aug 16, 2026
 *      Author: Tarun Kumar M
 */


#include <rf_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define GPIO_PIN_7 trig_pin
#define GPIO_PIN_6 echo_pin

extern TIM_HandleTypeDef htim3;
uint32_t  echo_val;
volatile uint32_t capture_count = 0;

void ultrasonic_task(INT stacd, void *exinf);

ID	tskid_2;			// Task ID number
T_CTSK ctsk_2 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= ultrasonic_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

float read_distance(void){
    float distance = 0.0343f * (echo_val / 2.0f);
    tm_printf((UB*)"duration = %d\r\n", echo_val);
    tm_printf((UB*)"distance = %d\r\n", (int)distance);
    tm_printf((UB*)"CNT = %lu, captures = %lu\r\n", __HAL_TIM_GET_COUNTER(&htim3),
              capture_count);
    return distance;
}

void ultrasonic_task(INT stacd, void *exinf){
    tm_printf((UB*)"Ultrasonic task started\r\n");

    if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        tm_printf((UB*)"IC start failed\r\n");
        Error_Handler();
    }

    while(1){
        read_distance();
        tk_dly_tsk(100);
    }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
	    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	    	echo_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	    	capture_count++;
	    }
	}



