/*
 * rf_task.h
 *
 *  Created on: Aug 5, 2026
 *      Author: Tarun Kumar M
 */

#ifndef INC_RF_TASK_H_
#define INC_RF_TASK_H_
#include <tk/tkernel.h>

extern ID	tskid_1;
extern T_CTSK ctsk_1;
void receive_rssi(void);
void rf_task1(INT stacd, void *exinf);

#endif /* INC_RF_TASK_H_ */
