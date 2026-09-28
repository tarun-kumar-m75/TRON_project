/*
 * ultrasonic_task.c
 *
 *  Created on: Aug 16, 2026
 *      Author: Tarun Kumar M
 */

#include <ultrasonic_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

extern TIM_HandleTypeDef htim3;

/* TIM3 tick = 1us, confirmed from clock config: HSI(64MHz)/HSIDiv2 =
 * 32MHz SYSCLK, APB1 prescaler=1 -> TIM3 clock = 32MHz, and
 * Prescaler = 32-1 in MX_TIM3_Init -> 32MHz/32 = 1MHz = 1 tick/us.
 * So echo_val below is directly in microseconds, matching the
 * 0.0343 cm/us formula in read_distance() without any extra scaling. */

#define TRIG_PULSE_TICKS   15   /* 15us pulse, comfortable margin over the 10us minimum */

/* Reject captured pulses outside this range before ever computing a
 * distance from them. TRIG (PA7) and ECHO (PA6) are physically adjacent
 * pins - a TRIG edge can capacitively couple onto the ECHO capture line
 * and register as a bogus near-zero-width pulse, which without this
 * filter reads as an implausibly close object and can permanently trip
 * obstacle avoidance. Upper bound covers the HC-SR04's real max range
 * (~400-450cm, ~26-38ms round trip) with headroom. */
#define MIN_VALID_PULSE_US   150
#define MAX_VALID_PULSE_US   25000

/* ultrasonic_task loops every 100ms - 5 loops with no completed echo
 * measurement = 500ms of silence, treated as "no fresh reading". */
#define STALE_LOOPS 5
#define MOTOR_LOOP_PERIOD_MS 100   /* matches this task's own tk_dly_tsk(100) below */

/* If the capture state machine is still waiting for a FALLING edge
 * after this many task-loop passes, the corresponding rising edge's
 * echo was lost entirely (no object in range, or a dropped edge) -
 * TRIG keeps auto-firing every ~50ms regardless, so without this
 * watchdog the state machine can desync and misread the NEXT cycle's
 * rising edge as this cycle's falling edge. One loop of slack (100ms)
 * comfortably covers the real max echo width (~38ms). */
#define MAX_STUCK_FALLING_LOOPS   1

typedef enum { WAIT_RISING, WAIT_FALLING } EdgeState;

static volatile EdgeState edge_state = WAIT_RISING;
static volatile uint32_t  echo_start = 0;
static volatile uint32_t  echo_val   = 0;   /* last COMPLETE, PLAUSIBLE pulse width, in us */
static volatile uint8_t   new_echo_ready = 0;
volatile uint32_t capture_count = 0;
volatile uint32_t rejected_count = 0;   /* pulses discarded by the plausibility filter - watch this in testing; if it climbs steadily, MIN/MAX_VALID_PULSE_US need retuning, not just crosstalk */

volatile uint8_t ultrasonic_lost = 1;
volatile float last_distance_cm = -1.0f;   /* was missing volatile - read cross-task from motor_task.c */
static uint32_t us_stale_counter = STALE_LOOPS;
static uint32_t stuck_falling_loops = 0;

ID	tskid_2;			// Task ID number
T_CTSK ctsk_2 = {				// Task creation information
	.itskpri	= 5,
	.stksz		= 1024,
	.task		= ultrasonic_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

/* Force the capture state machine back to a known-good state after a
 * missed echo. Brief critical section - the ISR can fire independently
 * of this task's priority (hardware IRQ always preempts), so the
 * state/polarity reset has to be atomic with respect to it. */
static void reset_capture_state(void){
	__disable_irq();
	edge_state = WAIT_RISING;
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	__enable_irq();
	stuck_falling_loops = 0;
}

float read_distance(void){
    /* Watchdog: catch a capture cycle that never got its falling edge. */
    if(edge_state == WAIT_FALLING){
        stuck_falling_loops++;
        if(stuck_falling_loops > MAX_STUCK_FALLING_LOOPS){
            tm_printf((UB*)"ultrasonic: WATCHDOG - stuck WAIT_FALLING, resetting capture state\r\n");
            reset_capture_state();
        }
    } else {
        stuck_falling_loops = 0;
    }

    if(new_echo_ready){
        new_echo_ready = 0;
        us_stale_counter = 0;
        ultrasonic_lost = 0;

        last_distance_cm = 0.0343f * (echo_val / 2.0f);

        tm_printf((UB*)"duration = %lu us\r\n", (unsigned long)echo_val);
        tm_printf((UB*)"distance = %d cm\r\n", (int)last_distance_cm);
        tm_printf((UB*)"CNT = %lu, captures = %lu, rejected = %lu\r\n",
                  __HAL_TIM_GET_COUNTER(&htim3), (unsigned long)capture_count,
                  (unsigned long)rejected_count);
    } else {
        /* No completed, PLAUSIBLE rising+falling edge pair this cycle. */
        if(us_stale_counter < 0xFFFFFFFFu) us_stale_counter++;
        if(us_stale_counter >= STALE_LOOPS){
            ultrasonic_lost = 1;
        }
        tm_printf((UB*)"ultrasonic: no fresh echo this cycle (captures=%lu, rejected=%lu)\r\n",
                  (unsigned long)capture_count, (unsigned long)rejected_count);
    }
    return last_distance_cm;
}

void ultrasonic_task(INT stacd, void *exinf){
    tm_printf((UB*)"Ultrasonic task started\r\n");

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TRIG_PULSE_TICKS);

    if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    /* Start listening for the echo's RISING edge first. */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    edge_state = WAIT_RISING;

    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        tm_printf((UB*)"IC start failed\r\n");
        Error_Handler();
    }

    while(1){
        read_distance();
        tk_dly_tsk(MOTOR_LOOP_PERIOD_MS);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_count++;

        if(edge_state == WAIT_RISING){
            /* This edge marks the START of the echo pulse. */
            echo_start = val;
            edge_state = WAIT_FALLING;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            /* This edge marks the END of the echo pulse - compute pulse
             * width, handling free-running counter wraparound
             * (period = 49999 ticks, ~50ms max). */
            uint32_t duration;
            if(val >= echo_start){
                duration = val - echo_start;
            } else {
                duration = (49999u - echo_start) + val + 1u;
            }

            /* Plausibility filter: discard anything too short to be a
             * real echo (TRIG/ECHO crosstalk) or too long to be within
             * the sensor's real range. A rejected reading does NOT set
             * new_echo_ready - read_distance() will see this as "no
             * fresh echo this cycle" rather than acting on a bad one. */
            if(duration >= MIN_VALID_PULSE_US && duration <= MAX_VALID_PULSE_US){
                echo_val = duration;
                new_echo_ready = 1;
            } else {
                rejected_count++;
            }

            edge_state = WAIT_RISING;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
