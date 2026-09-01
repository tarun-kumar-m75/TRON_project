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

#define buf_size 5

//#define huart1 &huart1
extern UART_HandleTypeDef huart1;
volatile uint8_t uart_rx_flag = 0;
uint8_t tail = 0;
volatile int8_t rssi_received;
 int8_t rssi_buffer[5];
 int8_t rssi_average = 0;
int sum = 0;
int count = 0;
int flag_127 = 0;

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
			/*if(rssi_buffer == -127){
				flag_127 = 1;
			}*/
			//else{
				sum = sum - rssi_buffer[tail] + rssi_received;
				rssi_buffer[tail] = rssi_received;
				tail = (tail + 1)%buf_size ;
				if(count < 5){
					count++;
				}
				uart_rx_flag = 0;

				if(count >= 5){
							rssi_average = sum/buf_size;
					}
				//}
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
			/*if(flag_127==1){
				tm_printf((UB*)"signal lost %d\r\n");
				flag_127 = 0;
			}*/
			tm_printf((UB*)"RSSI avg: %d\r\n", rssi_average);
			tk_dly_tsk(100);
	}
}
