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

extern TIM_HandleTypeDef htim3;

/* TIM3 tick = 1us, confirmed from clock config: HSI(64MHz)/HSIDiv2 =
 * 32MHz SYSCLK, APB1 prescaler=1 -> TIM3 clock = 32MHz, and
 * Prescaler = 32-1 in MX_TIM3_Init -> 32MHz/32 = 1MHz = 1 tick/us.
 * So echo_val below is directly in microseconds, matching the
 * 0.0343 cm/us formula in read_distance() without any extra scaling. */

/* HC-SR04 needs a >=10us HIGH trigger pulse. TIM_CHANNEL_2 (PA7) drives
 * this via hardware PWM in mode1, auto-repeating every timer period
 * (Period=49999 -> ~50ms) with NO CPU involvement once started - but
 * the pulse WIDTH is set by the channel's compare value (CCR2), which
 * was left at 0 in MX_TIM3_Init (sConfigOC.Pulse = 0). At 0% duty the
 * trigger pin never actually pulses, so the sensor was never being
 * triggered at all. Fixed by setting the compare value explicitly
 * before starting PWM output, below. */
#define TRIG_PULSE_TICKS   15   /* 15us pulse, comfortable margin over the 10us minimum */

/* ultrasonic_task loops every 100ms - 5 loops with no completed echo
 * measurement = 500ms of silence, treated as "no fresh reading". */
#define STALE_LOOPS 5

typedef enum { WAIT_RISING, WAIT_FALLING } EdgeState;

static volatile EdgeState edge_state = WAIT_RISING;
static volatile uint32_t  echo_start = 0;
static volatile uint32_t  echo_val   = 0;   /* last COMPLETE pulse width, in us */
static volatile uint8_t   new_echo_ready = 0;
volatile uint32_t capture_count = 0;

volatile uint8_t ultrasonic_lost = 1;
float last_distance_cm = -1.0f;
static uint32_t us_stale_counter = STALE_LOOPS;

void ultrasonic_task(INT stacd, void *exinf);   /* forward declaration - needed here because ctsk_2 below references it before its full definition later in the file */

ID	tskid_2;			// Task ID number
T_CTSK ctsk_2 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= ultrasonic_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

float read_distance(void){
    if(new_echo_ready){
        new_echo_ready = 0;
        us_stale_counter = 0;
        ultrasonic_lost = 0;

        last_distance_cm = 0.0343f * (echo_val / 2.0f);

        tm_printf((UB*)"duration = %lu us\r\n", (unsigned long)echo_val);
        tm_printf((UB*)"distance = %d cm\r\n", (int)last_distance_cm);
        tm_printf((UB*)"CNT = %lu, captures = %lu\r\n",
                  __HAL_TIM_GET_COUNTER(&htim3), (unsigned long)capture_count);
    } else {
        /* No completed rising+falling edge pair this cycle. */
        if(us_stale_counter < 0xFFFFFFFFu) us_stale_counter++;
        if(us_stale_counter >= STALE_LOOPS){
            ultrasonic_lost = 1;
        }
        tm_printf((UB*)"ultrasonic: no fresh echo this cycle (captures=%lu)\r\n",
                  (unsigned long)capture_count);
    }
    return last_distance_cm;
}

void ultrasonic_task(INT stacd, void *exinf){
    tm_printf((UB*)"Ultrasonic task started\r\n");

    /* FIX: set the trigger pulse width before starting PWM output -
     * previously this stayed at the init-time default of 0 (0% duty),
     * so the sensor was never actually triggered. */
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
        tk_dly_tsk(100);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_count++;

        if(edge_state == WAIT_RISING){
            /* This edge marks the START of the echo pulse. Record it
             * and switch to catching the FALLING edge next - this is
             * the two-phase measurement the original code was missing;
             * it previously read every captured edge the same way,
             * which returns a raw timer snapshot, not a duration. */
            echo_start = val;
            edge_state = WAIT_FALLING;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            /* This edge marks the END of the echo pulse - compute the
             * actual pulse width, handling free-running counter
             * wraparound (period = 49999 ticks, ~50ms max pulse). */
            uint32_t duration;
            if(val >= echo_start){
                duration = val - echo_start;
            } else {
                duration = (49999u - echo_start) + val + 1u;
            }

            echo_val = duration;
            new_echo_ready = 1;

            edge_state = WAIT_RISING;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
