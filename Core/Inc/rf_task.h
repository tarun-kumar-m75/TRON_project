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

/* Exposed for other tasks (e.g. motor_task) to read.
 * signal_lost covers two cases: the ESP32 explicitly sent the -127
 * "target lost" sentinel, OR no byte has arrived at all for a while
 * (link down / ESP32 crashed / cable unplugged). Either way, callers
 * should not trust rssi_average while signal_lost is set. */
extern volatile uint8_t signal_lost;
extern int8_t rssi_average;

/* Feature vector [mean, std, min, max, median] over the last 5 raw
 * RSSI bytes, matching the training pipeline's feature order exactly. */
extern float rssi_features[5];
extern volatile uint8_t features_valid;

/* Model output - this is what motor_task should actually read to
 * decide how to drive. distance_valid is cleared whenever signal_lost
 * is set, so motor_task never acts on a stale prediction. */
extern float target_distance_m;
extern volatile uint8_t distance_valid;

void receive_rssi(void);
void rf_task1(INT stacd, void *exinf);

#endif /* INC_RF_TASK_H_ */
