/*
 * motor_task.h
 *
 *  Created on: Sep 2, 2026
 *      Author: Tarun Kumar M
 */

#ifndef INC_MOTOR_TASK_H_
#define INC_MOTOR_TASK_H_

#include <tk/tkernel.h>

extern ID	tskid_3;
extern T_CTSK ctsk_3;

void motor_stop(void);
void motor_forward(uint16_t duty);
void motor_turn_right(uint16_t duty);
void motor_turn_left(uint16_t duty);
void motor_task(INT stacd, void *exinf);

#endif /* INC_MOTOR_TASK_H_ */
