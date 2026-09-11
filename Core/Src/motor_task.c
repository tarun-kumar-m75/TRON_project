/*
 * motor_task.c
 *
 *  Created on: Sep 2, 2026
 *      Author: Tarun Kumar M
 *
 * PRIORITY ORDER:
 *   1. Ultrasonic obstacle check - ALWAYS evaluated first, overrides
 *      everything else. If something is too close, or the ultrasonic
 *      reading itself is stale/missing, the rover stops and reroutes
 *      regardless of what the RSSI/distance model says.
 *   2. Only if the path is clear does RSSI-based approach behavior run.
 *
 * Turning: this rover has no separate steering axle - it's a
 * differential/skid-steer drive. Turning is done by spinning the two
 * sides in OPPOSITE directions (a pivot/tank turn).
 *
 * Obstacle handling now uses the ultrasonic-mounted servo sweep
 * (servo_task.c) to pick an INFORMED heading, instead of always
 * blindly turning the same fixed direction. RSSI itself is still not
 * direction-aware (antenna isn't on the servo) - this only improves
 * obstacle/path-finding, not signal bearing.
 */

#include <motor_task.h>
#include <rf_task.h>
#include <ultrasonic_task.h>
#include <servo_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

extern TIM_HandleTypeDef htim1;

/* Duty range assumes TIM1's Counter Period = 999 (1kHz PWM). */
#define MAX_DUTY            900
#define MIN_DUTY            250   /* TUNE on real hardware */

/* Distance thresholds, in meters - RSSI-based approach (TUNE on real testing). */
#define ARRIVE_THRESHOLD_M   0.5f
#define SLOWDOWN_START_M     3.0f

/* Obstacle avoidance (ultrasonic) - kept in sync with servo_task.c's
 * SWEEP_OBSTACLE_THRESHOLD_CM by hand for now. */
#define OBSTACLE_STOP_CM     25
#define TURN_DUTY            400
#define TURN_STEP_MS         300   /* open-loop turn time PER 45-degree
                                       sweep step - no encoders, so this
                                       is timing-based. TUNE by testing
                                       how far one step actually turns
                                       your specific rover. */

ID	tskid_3;
T_CTSK ctsk_3 = {
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= motor_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

/* ---------------- Low-level per-side wheel control ---------------- */

static void left_forward(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void left_reverse(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void right_forward(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

static void right_reverse(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

/* ---------------- Public movement functions ---------------- */

void motor_stop(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void motor_forward(uint16_t duty){
	if(duty > MAX_DUTY) duty = MAX_DUTY;
	left_forward(duty);
	right_forward(duty);
}

/* Pivot turn in place: one side forward, other side reverse.
 * Direction convention here is arbitrary - swap the two calls below if
 * your rover physically turns the opposite way from what's expected. */
void motor_turn_right(uint16_t duty){
	if(duty > MAX_DUTY) duty = MAX_DUTY;
	left_forward(duty);
	right_reverse(duty);
}

void motor_turn_left(uint16_t duty){
	if(duty > MAX_DUTY) duty = MAX_DUTY;
	left_reverse(duty);
	right_forward(duty);
}

/* ---------------- Main task ---------------- */

void motor_task(INT stacd, void *exinf){
	tm_printf((UB*)"Motor task started\r\n");

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();

	servo_init();
	motor_stop();

	while(1){
		/* PRIORITY 1: ultrasonic safety check - always evaluated first,
		 * overrides RSSI-based logic entirely when triggered. */
		if(ultrasonic_lost){
			motor_stop();
			tm_printf((UB*)"Motor: STOPPED (ultrasonic stale/no reading)\r\n");

		} else if(last_distance_cm >= 0 && last_distance_cm <= OBSTACLE_STOP_CM){
			/* Obstacle detected - stop, sweep to find the clearest
			 * heading, then turn PROPORTIONALLY toward it instead of
			 * always blindly turning the same fixed direction. */
			motor_stop();
			tm_printf((UB*)"Motor: OBSTACLE at %d cm - sweeping\r\n",
			          (int)last_distance_cm);
			tk_dly_tsk(100);

			int best_idx = servo_sweep_and_find_best();
			int center_idx = SERVO_NUM_ANGLES / 2;
			int offset = best_idx - center_idx;

			if(offset < 0){
				tm_printf((UB*)"Motor: turning LEFT (%d step(s))\r\n", -offset);
				motor_turn_left(TURN_DUTY);
				tk_dly_tsk((-offset) * TURN_STEP_MS);
				motor_stop();
			} else if(offset > 0){
				tm_printf((UB*)"Motor: turning RIGHT (%d step(s))\r\n", offset);
				motor_turn_right(TURN_DUTY);
				tk_dly_tsk(offset * TURN_STEP_MS);
				motor_stop();
			} else {
				tm_printf((UB*)"Motor: straight ahead is already clearest\r\n");
			}
			/* Loop back around - next iteration re-checks ultrasonic
			 * (now facing forward again, post-turn) before resuming
			 * RSSI-based approach. */

		} else {
			/* PRIORITY 2: path is clear - fall through to RSSI-based
			 * approach behavior. */
			if(!distance_valid || signal_lost){
				motor_stop();
				tm_printf((UB*)"Motor: stopped (no valid RSSI distance)\r\n");
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
		}

		tk_dly_tsk(100);
	}
}
