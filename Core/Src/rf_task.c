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
/* rf_task1 loops every 100ms (tk_dly_tsk(100)). 5 loops with no fresh
 * byte at all = 500ms of silence on the wire, matching the ESP32's own
 * LOST_TIMEOUT_MS. This is a software loop-counter, not a hardware
 * timer, since it only needs to be "good enough" to flag staleness -
 * exact wall-clock timing isn't required here. */
#define STALE_LOOPS 5

extern UART_HandleTypeDef huart1;
volatile uint8_t uart_rx_flag = 0;
uint8_t tail = 0;
volatile int8_t rssi_received;
int8_t rssi_buffer[buf_size];
int8_t rssi_average = 0;
int sum = 0;
int count = 0;

/* signal_lost starts at 1 (lost) until the first real byte arrives -
 * we don't want downstream logic trusting rssi_average=0 as if it were
 * a real reading before any data has come in at all. */
volatile uint8_t signal_lost = 1;
static uint32_t stale_counter = STALE_LOOPS;

/* Feature vector matching the training pipeline's FEATURE_COLS order:
 * [rssi_mean, rssi_std, rssi_min, rssi_max, rssi_median]. Computed from
 * the same 5-sample rssi_buffer already being kept for the average -
 * no new sampling logic needed, just more stats over the existing data. */
float rssi_features[5] = {0};
volatile uint8_t features_valid = 0;

/* Model output - what motor_task actually reads. */
float target_distance_m = -1.0f;
volatile uint8_t distance_valid = 0;

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

	target_distance_m = predict_distance(rssi_features);
	distance_valid = 1;
}

// task execution function
 ID	tskid_1;			// Task ID number
 T_CTSK ctsk_1 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= rf_task1,
	.tskatr		= TA_HLNG | TA_RNG3,
};


void receive_rssi(void){
	if(uart_rx_flag == 1){
		uart_rx_flag = 0;
		stale_counter = 0;   // a byte arrived this cycle, link is alive

		if(rssi_received == RSSI_LOST_SENTINEL){
			/* ESP32 explicitly reported "target lost". Don't fold this
			 * sentinel into the RSSI moving average - it isn't a real
			 * (very weak) reading, it's a flag. Just raise signal_lost
			 * and leave rssi_average/rssi_buffer untouched so the last
			 * known-good average is still visible if useful, but the
			 * flag tells callers not to trust it as current. */
			signal_lost = 1;
			distance_valid = 0;   /* don't let motor_task drive on a stale prediction */
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
		/* No byte at all this cycle. This is different from getting an
		 * explicit LOST sentinel - it could mean the UART link itself
		 * is down (unplugged, ESP32 crashed/reset) rather than just
		 * the BLE target being out of range. Either way, past a certain
		 * point of silence we shouldn't keep reporting a stale average
		 * as if it were current. */
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
