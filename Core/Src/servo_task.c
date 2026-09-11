/*
 * servo_task.c
 *
 *  Created on: Sep 11, 2026
 *      Author: Tarun Kumar M
 */

/*
 * servo_task.c
 *
 *  Created on: Sep 3, 2026
 *      Author: Tarun Kumar M
 */

#include <servo_task.h>
#include <ultrasonic_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

extern TIM_HandleTypeDef htim2;

/* Standard SG90-type pulse range: ~1000us = 0 degrees, ~2000us = 180
 * degrees, at 50Hz. TIM2 is configured for 1 tick = 1us (Prescaler=31,
 * Period=19999 -> 20ms period = 50Hz), so these values map directly to
 * the timer's compare register with no extra scaling. */
#define SERVO_MIN_US   1000
#define SERVO_MAX_US   2000

/* Time to let the servo physically finish moving AND let at least one
 * fresh ultrasonic trigger/echo cycle complete at the new angle before
 * trusting last_distance_cm - otherwise you'd read a stale value left
 * over from the PREVIOUS angle. TUNE if readings still look stale. */
#define SERVO_SETTLE_MS  200

/* Kept in sync with motor_task.c's OBSTACLE_STOP_CM by hand for now -
 * if you change one, change the other. Fine for a single-file duplicate
 * given project timeline; move to a shared header later if this grows. */
#define SWEEP_OBSTACLE_THRESHOLD_CM 25

const uint8_t sweep_angles[SERVO_NUM_ANGLES] = {0, 45, 90, 135, 180};
int16_t sweep_clearance_cm[SERVO_NUM_ANGLES];

void servo_init(void){
	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	servo_center();
}

void servo_set_angle(uint8_t angle_deg){
	if(angle_deg > 180) angle_deg = 180;
	uint32_t pulse_us = SERVO_MIN_US +
	                     ((uint32_t)angle_deg * (SERVO_MAX_US - SERVO_MIN_US)) / 180u;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

void servo_center(void){
	/* Middle entry of sweep_angles[] is treated as "straight ahead" -
	 * matches the chassis forward direction when the servo horn is
	 * mounted centered. */
	servo_set_angle(sweep_angles[SERVO_NUM_ANGLES / 2]);
}

int servo_sweep_and_find_best(void){
	int best_idx = 0;
	int16_t best_clearance = -2;   /* below -1, so even an all-unknown
	                                   sweep (-1 everywhere) still picks
	                                   index 0 as a fallback instead of
	                                   leaving best_idx undecided. */

	for(int i = 0; i < SERVO_NUM_ANGLES; i++){
		servo_set_angle(sweep_angles[i]);
		tk_dly_tsk(SERVO_SETTLE_MS);

		if(ultrasonic_lost){
			sweep_clearance_cm[i] = -1;
			tm_printf((UB*)"Sweep angle %d: no reading\r\n", sweep_angles[i]);
		} else {
			sweep_clearance_cm[i] = (int16_t)last_distance_cm;
			tm_printf((UB*)"Sweep angle %d: %d cm\r\n",
			          sweep_angles[i], sweep_clearance_cm[i]);
		}

		if(sweep_clearance_cm[i] > best_clearance){
			best_clearance = sweep_clearance_cm[i];
			best_idx = i;
		}
	}

	servo_center();   /* face forward again before the rover drives */

	if(best_clearance <= SWEEP_OBSTACLE_THRESHOLD_CM){
		tm_printf((UB*)"Sweep: every heading blocked/unknown - "
		          "best-effort angle=%d (%d cm)\r\n",
		          sweep_angles[best_idx], best_clearance);
	} else {
		tm_printf((UB*)"Sweep: clearest heading=%d deg (%d cm)\r\n",
		          sweep_angles[best_idx], best_clearance);
	}

	return best_idx;
}
