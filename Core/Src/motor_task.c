/*
 * motor_task.c
 *
 *  Created on: Sep 2, 2026
 *      Author: Tarun Kumar M
 *
 * FIRST MILESTONE BEHAVIOR: straight-line only, no steering.
 * Reads the ML model's predicted distance from rf_task (target_distance_m,
 * distance_valid), drives forward at a speed proportional to that
 * distance, slowing down as it closes in, and stopping once close
 * enough. No ultrasonic, no turning - that comes later.
 */

#include <motor_task.h>
#include <rf_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

extern TIM_HandleTypeDef htim1;

/* Duty range assumes TIM1's Counter Period = 999 (1kHz PWM, matching
 * the CubeMX fix). If you kept Period at a different value, scale
 * MAX_DUTY/MIN_DUTY to match - MAX_DUTY should equal (Period-1) roughly. */
#define MAX_DUTY            900   /* leave a little headroom under 999 */
#define MIN_DUTY             250   /* floor duty so the motors actually
                                      overcome static friction and move,
                                      instead of buzzing at very low duty -
                                      TUNE THIS on your actual rover; too
                                      low and it won't move, too high and
                                      it won't slow down enough near the target */

/* Distance thresholds, in meters - TUNE THESE based on real testing. */
#define ARRIVE_THRESHOLD_M   0.5f   /* stop once predicted distance is at or below this */
#define SLOWDOWN_START_M     3.0f   /* start slowing down within this range; full speed beyond it */

ID	tskid_3;
T_CTSK ctsk_3 = {
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= motor_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

void motor_stop(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	/* Direction pins left as-is - with duty=0 no current flows through
	 * the L298N regardless of IN pin state, so the motors coast to a
	 * stop safely either way. */
}

void motor_forward(uint16_t duty){
	if(duty > MAX_DUTY) duty = MAX_DUTY;

	/* Forward polarity on both sides - straight line only, no turning
	 * in this first milestone. */
	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

void motor_task(INT stacd, void *exinf){
	tm_printf((UB*)"Motor task started\r\n");

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();

	/* Start stopped - don't move until we have a real, valid prediction. */
	motor_stop();

	while(1){
		if(!distance_valid || signal_lost){
			motor_stop();
			tm_printf((UB*)"Motor: stopped (no valid distance)\r\n");
		} else if(target_distance_m <= ARRIVE_THRESHOLD_M){
			motor_stop();
			tm_printf((UB*)"Motor: arrived (dist=%d cm)\r\n",
			          (int)(target_distance_m * 100));
		} else {
			float ratio = (target_distance_m - ARRIVE_THRESHOLD_M) /
			              (SLOWDOWN_START_M - ARRIVE_THRESHOLD_M);
			if(ratio > 1.0f) ratio = 1.0f;
			if(ratio < 0.0f) ratio = 0.0f;

			uint16_t duty = (uint16_t)(MIN_DUTY + ratio * (MAX_DUTY - MIN_DUTY));
			motor_forward(duty);
			tm_printf((UB*)"Motor: forward duty=%u (dist=%d cm)\r\n",
			          duty, (int)(target_distance_m * 100));
		}

		tk_dly_tsk(100);
	}
}
