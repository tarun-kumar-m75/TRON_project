/*
 * ultrasonic_task.h
 *
 *  Created on: Aug 27, 2026
 *      Author: Tarun Kumar M
 */

#ifndef INC_ULTRASONIC_TASK_H_
#define INC_ULTRASONIC_TASK_H_

#include <tk/tkernel.h>

extern ID	tskid_2;
extern T_CTSK ctsk_2;
float read_distance(void);
void ultrasonic_task(INT stacd, void *exinf);

#endif /* INC_ULTRASONIC_TASK_H_ */
