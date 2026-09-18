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
 * Obstacle handling uses the ultrasonic-mounted servo sweep
 * (servo_task.c) to pick an INFORMED heading, instead of always
 * blindly turning the same fixed direction.
 *
 * Bearing-finding: RSSI from a single, roughly omnidirectional
 * antenna is a scalar (signal strength), not a vector - it can't tell
 * you a direction on its own. The antenna isn't servo-mounted (a
 * dipole whip swept through headings wouldn't discriminate angle any
 * better than leaving it fixed - see project chat), so bearing has to
 * come from comparing RSSI across CHASSIS headings: periodically pivot
 * through a ring of headings, sample filtered RSSI at each, and turn
 * to face whichever was strongest. This is the RSSI equivalent of
 * servo_sweep_and_find_best() below, just rotating the whole rover
 * instead of a servo horn.
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
                                       step - no encoders, so this is
                                       timing-based. TUNE by testing how
                                       far one step actually turns your
                                       specific rover. Reused below for
                                       bearing-sweep turns - same
                                       drivetrain, same calibration. */

/* ---------------- RSSI bearing sweep (chassis rotation) ---------------- */

#define BEARING_NUM_STEPS      8     /* headings sampled per sweep, evenly
                                         spaced (45 deg apart with 8 steps
                                         and TURN_STEP_MS per step). Fewer
                                         steps = faster sweep, coarser
                                         bearing; more steps = slower,
                                         finer. TUNE against how much
                                         RSSI actually swings by heading
                                         on your hardware (whip antenna -
                                         expect a small swing, a few dB). */
#define BEARING_SETTLE_MS      900   /* wait after each turn before
                                         trusting rssi_average - lets the
                                         old heading's samples flush out
                                         of rf_task.c's 5-sample buffer
                                         and fresh ones arrive from the
                                         new heading. TUNE UP if you see
                                         a heading's readings still look
                                         like the previous heading's. */
#define BEARING_SAMPLE_COUNT   4     /* extra polls of rssi_average per
                                         heading, averaged, on top of
                                         rf_task.c's own 5-sample
                                         average - the RSSI swing here is
                                         small, so extra averaging matters
                                         more than it does for the coarse
                                         anchor overrides in rf_task.c. */
#define BEARING_SAMPLE_GAP_MS  150
#define BEARING_LOST_RSSI      (-128)  /* worse than any real RSSI byte
                                           (int8_t range), so a heading
                                           with no valid signal never wins */

static int8_t bearing_rssi[BEARING_NUM_STEPS];

/* Pivot through BEARING_NUM_STEPS headings, sample filtered RSSI at
 * each, turn back to face whichever heading averaged strongest, and
 * return that step's index. Always turns the SAME direction (right)
 * while sweeping outward, then turns the opposite direction (left) to
 * return to the winning heading - assumes left/right pivot turns are
 * symmetric for the same duty/duration, matching the existing
 * obstacle-avoidance turn logic below. Re-check that assumption on
 * hardware if the rover ends up facing noticeably off from the
 * heading it printed as "best". */
static int bearing_sweep_and_find_best(void){
	int best_idx = 0;
	int8_t best_rssi = BEARING_LOST_RSSI;

	tm_printf((UB*)"Bearing: starting sweep (%d headings)\r\n", BEARING_NUM_STEPS);

	for(int i = 0; i < BEARING_NUM_STEPS; i++){
		if(i > 0){
			motor_turn_right(TURN_DUTY);
			tk_dly_tsk(TURN_STEP_MS);
			motor_stop();
		}
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

		int8_t heading_rssi = (valid_samples > 0)
		                       ? (int8_t)(sum / valid_samples)
		                       : BEARING_LOST_RSSI;
		bearing_rssi[i] = heading_rssi;

		tm_printf((UB*)"Bearing step %d: rssi=%d (%d/%d valid samples)\r\n",
		          i, heading_rssi, valid_samples, BEARING_SAMPLE_COUNT);

		if(heading_rssi > best_rssi){
			best_rssi = heading_rssi;
			best_idx = i;
		}
	}

	/* We're now physically at the LAST sampled heading
	 * (BEARING_NUM_STEPS-1 turn-steps clockwise from where the sweep
	 * started). Turn back (counter-clockwise) to face best_idx. */
	int steps_back = (BEARING_NUM_STEPS - 1) - best_idx;
	if(steps_back > 0){
		motor_turn_left(TURN_DUTY);
		tk_dly_tsk(steps_back * TURN_STEP_MS);
		motor_stop();
	}

	tm_printf((UB*)"Bearing: best heading step=%d (rssi=%d) - facing it now\r\n",
	          best_idx, best_rssi);
	return best_idx;
}

/* How often to re-run the bearing sweep while actively approaching.
 * Counted in motor_task's own 100ms loop period, only while driving
 * forward toward the target (not while stopped/arrived/avoiding an
 * obstacle) - see ms_since_bearing_sweep below.
 * Too short: wastes time re-sweeping instead of covering ground.
 * Too long: the rover can drift/drive past a heading change (target
 * moved, or the rover's own heading drifted) before correcting.
 * TUNE against how fast your target/rover actually move. */
#define BEARING_SWEEP_INTERVAL_MS   4000
#define MOTOR_LOOP_PERIOD_MS        100

/* Initialized already-due so the FIRST time the rover has a valid
 * RSSI distance and a clear path, it sweeps for a bearing before ever
 * committing to a blind forward drive - this is what was missing
 * before (rover only ever knew "how far", never "which way"). */
static uint32_t ms_since_bearing_sweep = BEARING_SWEEP_INTERVAL_MS;

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
			/* Something is close. Before treating this as an obstacle
			 * to avoid, check whether the RSSI model ALSO says we're
			 * very close to the target - if so, this close object is
			 * almost certainly the target itself (e.g. the phone), not
			 * an unrelated obstacle, and we should stop, not reroute
			 * around it. */
			if(distance_valid && !signal_lost && target_distance_m <= ARRIVE_THRESHOLD_M){
				motor_stop();
				tm_printf((UB*)"Motor: ARRIVED (ultrasonic %d cm + RSSI %d cm agree)\r\n",
				          (int)last_distance_cm, (int)(target_distance_m * 100));

			} else {
				/* Genuine obstacle, unrelated to the target - stop,
				 * sweep to find the clearest heading, then turn
				 * PROPORTIONALLY toward it. */
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
				/* An obstacle-avoidance turn changed our heading - force
				 * a fresh bearing sweep before driving forward again
				 * instead of trusting whatever heading was "best" before
				 * we turned to dodge something. */
				ms_since_bearing_sweep = BEARING_SWEEP_INTERVAL_MS;
			}

		} else {
			/* PRIORITY 2: path is clear - fall through to RSSI-based
			 * approach behavior. */
			if(!distance_valid || signal_lost){
				motor_stop();
				tm_printf((UB*)"Motor: stopped (no valid RSSI distance)\r\n");
				/* No signal to bear on right now - re-sweep as soon as
				 * it comes back instead of resuming a stale heading. */
				ms_since_bearing_sweep = BEARING_SWEEP_INTERVAL_MS;

			} else if(target_distance_m <= ARRIVE_THRESHOLD_M){
				motor_stop();
				tm_printf((UB*)"Motor: arrived (dist=%d cm)\r\n",
				          (int)(target_distance_m * 100));

			} else if(ms_since_bearing_sweep >= BEARING_SWEEP_INTERVAL_MS){
				ms_since_bearing_sweep = 0;
				bearing_sweep_and_find_best();
				/* Next loop iteration re-checks ultrasonic/RSSI and
				 * drives forward along the now-corrected heading. */

			} else {
				ms_since_bearing_sweep += MOTOR_LOOP_PERIOD_MS;

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

		tk_dly_tsk(MOTOR_LOOP_PERIOD_MS);
	}
}
