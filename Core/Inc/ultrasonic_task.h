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

/* Exposed for other tasks (e.g. the future navigation/obstacle-avoidance
 * task) to read. ultrasonic_lost mirrors the same idea as rf_task's
 * signal_lost: don't trust last_distance_cm if this is set. */
extern volatile uint8_t ultrasonic_lost;
extern float last_distance_cm;

float read_distance(void);
void ultrasonic_task(INT stacd, void *exinf);

#endif /* INC_ULTRASONIC_TASK_H_ */
