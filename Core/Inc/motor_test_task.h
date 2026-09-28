#ifndef MOTOR_TEST_TASK_H_
#define MOTOR_TEST_TASK_H_

#include <tk/tkernel.h>

extern ID     tskid_test;
extern T_CTSK ctsk_test;

void motor_test_task(INT stacd, void *exinf);

#endif /* MOTOR_TEST_TASK_H_ */
