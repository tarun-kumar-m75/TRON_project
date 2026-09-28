/*
 * motor_task.c
 *
 *  Created on: Sep 2, 2026
 *      Author: Tarun Kumar M
 *
 * PRIORITY ORDER:
 *   1. Ultrasonic obstacle check - ALWAYS evaluated first, overrides
 *      everything else. (Task priority: ultrasonic_task=5 > motor_task=10
 *      > rf_task1=15.)
 *   2. Path clear -> RSSI-based approach, two modes:
 *        - COARSE (target_distance_m > COARSE_THRESHOLD_M): HOT/COLD
 *          search comparing each bearing sweep's best raw RSSI to the
 *          previous sweep's.
 *        - PRECISION (target_distance_m <= COARSE_THRESHOLD_M): use the
 *          model's distance for proportional slowdown and arrival.
 *
 * WALL vs OBSTACLE during the bearing sweep: at each heading, if
 * ultrasonic sees something within OBSTACLE_STOP_CM, the sweep takes a
 * second reading one step further in the same rotation direction:
 *   - second reading <= first (same or closer) -> WALL: exclude this
 *     heading, then RETURN TO THE SWEEP'S STARTING HEADING before
 *     continuing in the reversed direction. Without this return trip,
 *     simply flipping direction from the blocked position just re-treads
 *     the arc already sampled instead of ever reaching the unexplored
 *     far side of the circle - which is exactly the "only covers half
 *     the circle" bug this fixes.
 *   - second reading > first AND > OBSTACLE_STOP_CM (now clear) ->
 *     just an OBSTACLE: sample RSSI at this now-clear heading and
 *     continue the sweep in the same direction as before.
 */

#include <motor_task.h>
#include <rf_task.h>
#include <ultrasonic_task.h>
#include <servo_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

extern TIM_HandleTypeDef htim1;

#define MAX_DUTY            600
#define MIN_DUTY            250

#define ARRIVE_THRESHOLD_M   0.5f
#define SLOWDOWN_START_M     3.0f
#define COARSE_THRESHOLD_M   5.0f
#define COARSE_CRUISE_DUTY   450

#define OBSTACLE_STOP_CM     25
#define TURN_DUTY            350
#define TURN_STEP_MS         300

/* ---------------- RSSI bearing sweep (chassis rotation) ---------------- */

#define BEARING_NUM_STEPS      8
#define BEARING_SETTLE_MS      900
#define BEARING_SAMPLE_COUNT   4
#define BEARING_SAMPLE_GAP_MS  150
#define BEARING_LOST_RSSI      (-128)

#define BEARING_SWEEP_INTERVAL_MS       4000
#define BEARING_COLD_SWEEP_INTERVAL_MS  1500
#define MOTOR_LOOP_PERIOD_MS            100

/* Once a direction reversal has already happened, a SECOND wall hit
 * means both sides are genuinely blocked within budget - stop trying
 * to reverse again (which would just re-cross ground already covered
 * a second time) and sample whatever's left in place. */
#define MAX_DIRECTION_REVERSALS   1

typedef struct {
	int8_t rssi;
	int8_t net_rotation;
} bearing_sample_t;

static bearing_sample_t bearing_samples[BEARING_NUM_STEPS];

static void turn_signed_steps(int steps){
	if(steps > 0){
		motor_turn_right(TURN_DUTY);
		tk_dly_tsk(steps * TURN_STEP_MS);
		motor_stop();
	} else if(steps < 0){
		motor_turn_left(TURN_DUTY);
		tk_dly_tsk((-steps) * TURN_STEP_MS);
		motor_stop();
	}
}

static int obstacle_ahead(void){
	return (!ultrasonic_lost && last_distance_cm >= 0 && last_distance_cm <= OBSTACLE_STOP_CM);
}

static int8_t sample_rssi_here(void){
	tk_dly_tsk(BEARING_SETTLE_MS);

	int32_t sum = 0;
	int valid_samples = 0;
	for(int s = 0; s < BEARING_SAMPLE_COUNT; s++){
		if(!signal_lost && distance_valid){
			sum += rssi_average;
			valid_samples++;
		}
		tk_dly_tsk(BEARING_SAMPLE_GAP_MS);
	}

	return (valid_samples > 0) ? (int8_t)(sum / valid_samples) : BEARING_LOST_RSSI;
}

static int bearing_sweep_and_find_best(int8_t *out_best_rssi){
	int direction = +1;
	int net_rotation = 0;
	int sampled_count = 0;
	int best_slot = 0;
	int8_t best_rssi = BEARING_LOST_RSSI;
	int reversals_used = 0;

	tm_printf((UB*)"Bearing: starting sweep (%d heading slots)\r\n", BEARING_NUM_STEPS);

	while(sampled_count < BEARING_NUM_STEPS){
		if(sampled_count > 0){
			turn_signed_steps(direction);
			net_rotation += direction;
		}

		if(obstacle_ahead()){
			int16_t first_reading = last_distance_cm;

			tm_printf((UB*)"Bearing: obstacle at net_rot=%d (%d cm) - probing further\r\n",
			          net_rotation, (int)first_reading);
			turn_signed_steps(direction);
			net_rotation += direction;

			int obstacle_gone = (!ultrasonic_lost &&
			                      last_distance_cm > first_reading &&
			                      last_distance_cm > OBSTACLE_STOP_CM);

			if(obstacle_gone){
				tm_printf((UB*)"Bearing: %d cm -> %d cm, clear beyond - OBSTACLE, sampling here\r\n",
				          (int)first_reading, (int)last_distance_cm);
				/* fall through: sample RSSI here, continue same direction */
			} else {
				int confirmed_wall = (!ultrasonic_lost && last_distance_cm <= first_reading);

				tm_printf((UB*)"Bearing: %d cm -> %d cm, %s - excluding this slot\r\n",
				          (int)first_reading, (int)last_distance_cm,
				          confirmed_wall ? "WALL confirmed" : "still ambiguous");

				bearing_samples[sampled_count].rssi = BEARING_LOST_RSSI;
				bearing_samples[sampled_count].net_rotation = (int8_t)net_rotation;
				sampled_count++;

				if(confirmed_wall && reversals_used < MAX_DIRECTION_REVERSALS){
					/* Return to the sweep's STARTING heading before
					 * reversing, so the reversed direction explores the
					 * genuinely unvisited far side of the circle instead
					 * of re-treading the arc already sampled between
					 * here and the start. This is the fix - without the
					 * return-to-origin step, reversing in place only
					 * ever covers about half the circle. */
					tm_printf((UB*)"Bearing: WALL - returning to start (net_rot %d -> 0) before reversing\r\n",
					          net_rotation);
					turn_signed_steps(0 - net_rotation);
					net_rotation = 0;
					direction = -direction;
					reversals_used++;
				}
				/* else: no reversal budget left (or unconfirmed) - keep
				 * current direction/position, just skip this slot. */
				continue;
			}
		}

		int8_t heading_rssi = sample_rssi_here();
		bearing_samples[sampled_count].rssi = heading_rssi;
		bearing_samples[sampled_count].net_rotation = (int8_t)net_rotation;

		tm_printf((UB*)"Bearing slot %d: rssi=%d net_rot=%d\r\n",
		          sampled_count, heading_rssi, net_rotation);

		if(heading_rssi > best_rssi){
			best_rssi = heading_rssi;
			best_slot = sampled_count;
		}
		sampled_count++;
	}

	int best_net_rotation = bearing_samples[best_slot].net_rotation;
	turn_signed_steps(best_net_rotation - net_rotation);

	tm_printf((UB*)"Bearing: best heading net_rot=%d (rssi=%d) - facing it now\r\n",
	          best_net_rotation, best_rssi);

	if(out_best_rssi) *out_best_rssi = best_rssi;
	return best_slot;
}

static uint32_t ms_since_bearing_sweep = BEARING_SWEEP_INTERVAL_MS;
static uint32_t current_sweep_interval_ms = BEARING_SWEEP_INTERVAL_MS;
static int8_t last_sweep_best_rssi = BEARING_LOST_RSSI;

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
		if(ultrasonic_lost){
			motor_stop();
			tm_printf((UB*)"Motor: STOPPED (ultrasonic stale/no reading)\r\n");

		} else if(last_distance_cm >= 0 && last_distance_cm <= OBSTACLE_STOP_CM){
			if(distance_valid && !signal_lost && target_distance_m <= ARRIVE_THRESHOLD_M){
				motor_stop();
				tm_printf((UB*)"Motor: ARRIVED (ultrasonic %d cm + RSSI %d cm agree) - HALTED\r\n",
				          (int)last_distance_cm, (int)(target_distance_m * 100));

			} else {
				motor_stop();
				tm_printf((UB*)"Motor: OBSTACLE at %d cm - sweeping\r\n",
				          (int)last_distance_cm);
				tk_dly_tsk(100);

				int best_idx = servo_sweep_and_find_best();
				int center_idx = SERVO_NUM_ANGLES / 2;
				int offset = best_idx - center_idx;

				if(offset < 0){
					motor_turn_left(TURN_DUTY);
					tk_dly_tsk((-offset) * TURN_STEP_MS);
					motor_stop();
				} else if(offset > 0){
					motor_turn_right(TURN_DUTY);
					tk_dly_tsk(offset * TURN_STEP_MS);
					motor_stop();
				}
				ms_since_bearing_sweep = current_sweep_interval_ms;
			}

		} else {
			if(!distance_valid || signal_lost){
				motor_stop();
				tm_printf((UB*)"Motor: stopped (no valid RSSI distance)\r\n");
				ms_since_bearing_sweep = current_sweep_interval_ms;

			} else if(target_distance_m <= ARRIVE_THRESHOLD_M){
				motor_stop();
				tm_printf((UB*)"Motor: ARRIVED (dist=%d cm) - HALTED\r\n",
				          (int)(target_distance_m * 100));

			} else if(ms_since_bearing_sweep >= current_sweep_interval_ms){
				ms_since_bearing_sweep = 0;

				int8_t new_best_rssi;
				bearing_sweep_and_find_best(&new_best_rssi);

				if(target_distance_m > COARSE_THRESHOLD_M){
					if(last_sweep_best_rssi == BEARING_LOST_RSSI ||
					   new_best_rssi > last_sweep_best_rssi){
						tm_printf((UB*)"Bearing: HOT (rssi %d > prev %d) - cruising\r\n",
						          new_best_rssi, last_sweep_best_rssi);
						current_sweep_interval_ms = BEARING_SWEEP_INTERVAL_MS;
					} else {
						tm_printf((UB*)"Bearing: COLD (rssi %d <= prev %d) - re-checking sooner\r\n",
						          new_best_rssi, last_sweep_best_rssi);
						current_sweep_interval_ms = BEARING_COLD_SWEEP_INTERVAL_MS;
					}
					last_sweep_best_rssi = new_best_rssi;
				} else {
					current_sweep_interval_ms = BEARING_SWEEP_INTERVAL_MS;
					last_sweep_best_rssi = BEARING_LOST_RSSI;
				}

			} else if(target_distance_m > COARSE_THRESHOLD_M){
				ms_since_bearing_sweep += MOTOR_LOOP_PERIOD_MS;
				motor_forward(COARSE_CRUISE_DUTY);
				tm_printf((UB*)"Motor: COARSE cruise duty=%u (dist=%d cm)\r\n",
				          COARSE_CRUISE_DUTY, (int)(target_distance_m * 100));

			} else {
				ms_since_bearing_sweep += MOTOR_LOOP_PERIOD_MS;

				float ratio = (target_distance_m - ARRIVE_THRESHOLD_M) /
				              (SLOWDOWN_START_M - ARRIVE_THRESHOLD_M);
				if(ratio > 1.0f) ratio = 1.0f;
				if(ratio < 0.0f) ratio = 0.0f;

				uint16_t duty = (uint16_t)(MIN_DUTY + ratio * (MAX_DUTY - MIN_DUTY));
				motor_forward(duty);
				tm_printf((UB*)"Motor: PRECISION forward duty=%u (dist=%d cm)\r\n",
				          duty, (int)(target_distance_m * 100));
			}
		}

		tk_dly_tsk(MOTOR_LOOP_PERIOD_MS);
	}
}
