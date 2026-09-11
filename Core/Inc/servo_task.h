/*
 * servo_task.h
 *
 *  Created on: Sep 3, 2026
 *      Author: Tarun Kumar M
 *
 * NOTE: despite the filename (kept consistent with rf_task/ultrasonic_task/
 * motor_task naming), this is NOT a separate uT-Kernel task - no T_CTSK/ID
 * here. A sweep is inherently a blocking, sequential operation (move,
 * settle, read, repeat) with only one consumer (motor_task), so it's
 * just driver functions called synchronously from motor_task's obstacle
 * branch - no benefit to running it as its own concurrent task.
 */

#ifndef INC_SERVO_TASK_H_
#define INC_SERVO_TASK_H_

#include <tk/tkernel.h>

#define SERVO_NUM_ANGLES 5

/* Angle convention (TUNE/VERIFY on real hardware - servo horn orientation
 * determines which physical side 0 vs 180 actually corresponds to; swap
 * the array values below if left/right come out backwards on first test). */
extern const uint8_t sweep_angles[SERVO_NUM_ANGLES];   /* e.g. {0,45,90,135,180} */
extern int16_t sweep_clearance_cm[SERVO_NUM_ANGLES];   /* -1 = no/stale reading at that angle */

void servo_init(void);
void servo_set_angle(uint8_t angle_deg);
void servo_center(void);

/* Sweeps through all angles, records clearance at each, re-centers the
 * servo afterward, and returns the index (into sweep_angles[]/
 * sweep_clearance_cm[]) of the clearest heading found. Always returns a
 * valid index (0..SERVO_NUM_ANGLES-1) even if every heading was blocked
 * or unreadable - in that case it returns a best-effort (least-bad)
 * choice rather than failing, so the caller always has something to act
 * on. This is a simple heuristic, not true dead-end backtracking. */
int servo_sweep_and_find_best(void);

#endif /* INC_SERVO_TASK_H_ */
