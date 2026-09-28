/*
 * rf_task.c
 *
 *  Created on: Aug 1, 2026
 *      Author: Tarun Kumar M
 */

#include <rf_task.h>
#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <math.h>
#include "model_predict.h"

#define buf_size 5
#define RSSI_LOST_SENTINEL -127
#define STALE_LOOPS 5

/* ---------------- Raw-RSSI anchor overrides ---------------------------- */
#define RSSI_STRONG_DBM   -66   /* stronger (less negative) than this -> anchor CLOSE */
#define RSSI_WEAK_DBM     -82   /* weaker (more negative) than this -> anchor FAR */
#define ANCHOR_CLOSE_M    0.3f
#define ANCHOR_FAR_M      5.0f

extern UART_HandleTypeDef huart1;
volatile uint8_t uart_rx_flag = 0;
uint8_t tail = 0;
volatile int8_t rssi_received;
int8_t rssi_buffer[buf_size];
int8_t rssi_average = 0;
int sum = 0;
int count = 0;

volatile uint8_t signal_lost = 1;
static uint32_t stale_counter = STALE_LOOPS;

float rssi_features[5] = {0};
volatile uint8_t features_valid = 0;

float target_distance_m = -1.0f;
volatile uint8_t distance_valid = 0;

static float apply_anchor_override(float model_prediction, int8_t rssi){
	if(rssi >= RSSI_STRONG_DBM){
		return ANCHOR_CLOSE_M;
	} else if(rssi <= RSSI_WEAK_DBM){
		return ANCHOR_FAR_M;
	}
	return model_prediction;
}

static void compute_features_and_predict(void){
	int8_t sorted[buf_size];
	int8_t minV = 127, maxV = -128;
	float sumF = 0;

	for(int i = 0; i < buf_size; i++){
		sorted[i] = rssi_buffer[i];
		sumF += rssi_buffer[i];
		if(rssi_buffer[i] < minV) minV = rssi_buffer[i];
		if(rssi_buffer[i] > maxV) maxV = rssi_buffer[i];
	}
	float mean = sumF / buf_size;

	float sqSum = 0;
	for(int i = 0; i < buf_size; i++){
		float d = rssi_buffer[i] - mean;
		sqSum += d * d;
	}
	float stddev = sqrtf(sqSum / buf_size);

	for(int i = 1; i < buf_size; i++){
		int8_t key = sorted[i];
		int j = i - 1;
		while(j >= 0 && sorted[j] > key){ sorted[j+1] = sorted[j]; j--; }
		sorted[j+1] = key;
	}
	int8_t median = sorted[buf_size / 2];

	rssi_features[0] = mean;
	rssi_features[1] = stddev;
	rssi_features[2] = (float)minV;
	rssi_features[3] = (float)maxV;
	rssi_features[4] = (float)median;
	features_valid = 1;

	float model_prediction = predict_distance(rssi_features);
	target_distance_m = apply_anchor_override(model_prediction, rssi_average);
	distance_valid = 1;
}

// task execution function
 ID	tskid_1;			// Task ID number
 T_CTSK ctsk_1 = {				// Task creation information
	.itskpri	= 15,   /* lowest of the three tasks - ultrasonic (5) and
	                       motor (10) both take priority over RF, since
	                       obstacle safety and motor reaction matter more
	                       than shaving a few hundred ms off RSSI freshness */
	.stksz		= 1024,
	.task		= rf_task1,
	.tskatr		= TA_HLNG | TA_RNG3,
};


void receive_rssi(void){
	if(uart_rx_flag == 1){
		uart_rx_flag = 0;
		stale_counter = 0;

		if(rssi_received == RSSI_LOST_SENTINEL){
			signal_lost = 1;
			distance_valid = 0;
			return;
		}

		signal_lost = 0;
		sum = sum - rssi_buffer[tail] + rssi_received;
		rssi_buffer[tail] = rssi_received;
		tail = (tail + 1) % buf_size;
		if(count < buf_size){
			count++;
		}
		if(count >= buf_size){
			rssi_average = sum / buf_size;
			compute_features_and_predict();
		}
	} else {
		if(stale_counter < 0xFFFFFFFFu) stale_counter++;
		if(stale_counter >= STALE_LOOPS){
			signal_lost = 1;
			distance_valid = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		uart_rx_flag = 1;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&rssi_received, 1);
	}

}

 void rf_task1(INT stacd, void *exinf)
{
	tm_printf((UB*)"RF task started\r\n");
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rssi_received, 1);
	while(1){
		receive_rssi();
		if(signal_lost){
			tm_printf((UB*)"RSSI: target lost / link stale\r\n");
		} else if(distance_valid){
			tm_printf((UB*)"RSSI avg: %d | predicted distance: %d cm\r\n",
			          rssi_average, (int)(target_distance_m * 100));
		} else {
			tm_printf((UB*)"RSSI avg: %d | filling buffer...\r\n", rssi_average);
		}
		tk_dly_tsk(100);
	}
}
