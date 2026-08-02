/*
 * rf_task.c
 *
 *  Created on: Aug 1, 2026
 *      Author: Tarun Kumar M
 */

#include "main.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define buf_size 5

//#define huart1 &huart1
extern UART_HandleTypeDef huart2;
volatile uint8_t uart_rx_flag = 0;
uint8_t tail = 0;
volatile int8_t rssi_received;
 int8_t rssi_buffer[5];
 int8_t rssi_average = 0;
int sum = 0;
int count = 0;

LOCAL void rf_task1(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_1;			// Task ID number
LOCAL T_CTSK ctsk_1 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= rf_task1,
	.tskatr		= TA_HLNG | TA_RNG3,
};


void receive_rssi(void){
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rssi_received, 1);

		if(uart_rx_flag == 1){
			sum = sum - rssi_buffer[tail] + rssi_received;
			rssi_buffer[tail] = rssi_received;
			tail = (tail + 1)%buf_size ;
			if(count < 5){
				count++;
			}
			uart_rx_flag = 0;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rssi_received, 1);
			if(count >= 5){
						rssi_average = sum/buf_size;
				}
			}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		uart_rx_flag = 1;
	}

}

LOCAL void rf_task1(INT stacd, void *exinf)
{
	while(1){
	receive_rssi();
	tm_printf((UB*)rssi_average);
	tk_dly_tsk(100);
	}
}
