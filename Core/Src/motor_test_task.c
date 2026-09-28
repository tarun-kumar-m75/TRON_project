/*
 * motor_test_task.c
 *
 * Hardware bring-up task: cycles through each wheel individually, then
 * both together, so you can watch/listen to exactly one stage at a time
 * and know immediately which motor (if any) isn't behaving.
 *
 * Runs as a REAL uT-Kernel task (not called from main() before the
 * kernel starts) - uses tk_dly_tsk() instead of HAL_Delay() so it
 * yields properly to the scheduler instead of blocking the CPU.
 *
 * Pin config (matches your CubeMX pinout diagram):
 *   MOTOR_IN1 = PC0      MOTOR_IN3 = PC2
 *   MOTOR_IN2 = PC1      MOTOR_IN4 = PC3
 *   TIM1_CH1  = PA8  (left side PWM)
 *   TIM1_CH2  = PA9  (right side PWM)
 *
 * TEMPORARY USE: in app_main.c, this replaces rf_task1/ultrasonic_task/
 * motor_task for now (those three are commented out there) - swap back
 * once bring-up testing is done.
 */

#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "motor_test_task.h"   /* declares motor_test_task() BEFORE it's
                                   referenced in ctsk_test's initializer
                                   below - without this include, the
                                   struct init sees an undeclared symbol
                                   since the function itself isn't
                                   defined until further down the file */

extern TIM_HandleTypeDef htim1;

#define MOTOR_IN1_PORT   GPIOC
#define MOTOR_IN1_PIN    GPIO_PIN_0
#define MOTOR_IN2_PORT   GPIOC
#define MOTOR_IN2_PIN    GPIO_PIN_1
#define MOTOR_IN3_PORT   GPIOC
#define MOTOR_IN3_PIN    GPIO_PIN_2
#define MOTOR_IN4_PORT   GPIOC
#define MOTOR_IN4_PIN    GPIO_PIN_3

#define TEST_DUTY        300
#define TEST_RUN_MS      1500
#define TEST_PAUSE_MS    800
#define TEST_CYCLE_PAUSE_MS   3000

ID     tskid_test;
T_CTSK ctsk_test = {
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= motor_test_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

static void test_stop(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN3_PORT, MOTOR_IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN4_PORT, MOTOR_IN4_PIN, GPIO_PIN_RESET);
}

static void test_left_forward(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void test_left_reverse(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void test_right_forward(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN3_PORT, MOTOR_IN3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN4_PORT, MOTOR_IN4_PIN, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

static void test_right_reverse(uint16_t duty){
	HAL_GPIO_WritePin(MOTOR_IN3_PORT, MOTOR_IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN4_PORT, MOTOR_IN4_PIN, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

void motor_test_task(INT stacd, void *exinf){
	tm_printf((UB*)"motor_test_task: started\r\n");

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

	test_stop();

	while(1){
		tm_printf((UB*)"motor_test: LEFT forward\r\n");
		test_left_forward(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();
		tk_dly_tsk(TEST_PAUSE_MS);

		tm_printf((UB*)"motor_test: LEFT reverse\r\n");
		test_left_reverse(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();
		tk_dly_tsk(TEST_PAUSE_MS);

		tm_printf((UB*)"motor_test: RIGHT forward\r\n");
		test_right_forward(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();
		tk_dly_tsk(TEST_PAUSE_MS);

		tm_printf((UB*)"motor_test: RIGHT reverse\r\n");
		test_right_reverse(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();
		tk_dly_tsk(TEST_PAUSE_MS);

		tm_printf((UB*)"motor_test: BOTH forward (straight)\r\n");
		test_left_forward(TEST_DUTY);
		test_right_forward(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();
		tk_dly_tsk(TEST_PAUSE_MS);

		tm_printf((UB*)"motor_test: PIVOT turn\r\n");
		test_left_forward(TEST_DUTY);
		test_right_reverse(TEST_DUTY);
		tk_dly_tsk(TEST_RUN_MS);
		test_stop();

		tm_printf((UB*)"motor_test: cycle complete, repeating\r\n");
		tk_dly_tsk(TEST_CYCLE_PAUSE_MS);
	}
}
