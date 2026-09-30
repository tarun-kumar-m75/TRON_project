# System Architecture and Operational Documentation

![architecture](./Photos/rescue_rover_system_architecture.png)


## 1. Purpose

This document describes the system architecture, task-level design, and
operational logic of the rescue rover. It is an autonomous rover
designed to locate a target carrying a Bluetooth Low Energy (BLE)
beacon by measuring received signal strength (RSSI), navigate toward
the target while avoiding obstacles, and halt upon arrival.

---

## 2. System Architecture

The system is composed of two microcontrollers, each responsible for a
distinct functional domain.

| Subsystem | Component | Responsibility |
|---|---|---|
| BLE acquisition | ESP32 | BLE scanning, RSSI filtering, data forwarding |
| Real-time control | STM32H533RE | Distance estimation, obstacle detection, motor control |

### 2.1 ESP32 (BLE Acquisition Layer)

The ESP32 performs continuous BLE scanning using the NimBLE stack and
identifies the target beacon by its service UUID. Received signal
strength values are processed through a two-stage filter:

1. A median filter, to remove transient outlier readings.
2. A Kalman filter, to produce a smoothed estimate of signal strength
   over time.

The filtered RSSI value is transmitted to the STM32 as a single byte
per sampling interval over UART. If the beacon is not detected within
a defined timeout period, a reserved sentinel value is transmitted in
place of a valid reading to explicitly indicate signal loss.

### 2.2 STM32H533RE (Real-Time Control Layer)

The STM32H533RE executes all time-critical operations under μT-Kernel
3.0, a real-time operating system. Functionality is partitioned into
three independent tasks rather than a single control loop, to ensure
that obstacle detection and response are not delayed by RSSI
processing or distance estimation.

---

## 3. Task Architecture

Three tasks are created and started from the application entry point
(`app_main.c`). Task priority determines scheduling precedence when
multiple tasks are ready to execute simultaneously.

| Task | Source File | Priority (lower value = higher priority) | Function |
|---|---|---|---|
| `ultrasonic_task` | `ultrasonic_task.c` | 5 | Obstacle distance measurement |
| `motor_task` | `motor_task.c` | 10 | Navigation and motor control |
| `rf_task1` | `rf_task.c` | 15 | RSSI reception and distance prediction |

Priority assignment reflects the relative cost of latency in each
subsystem: a delay in obstacle sensing carries a direct safety
consequence, whereas a delay in RSSI processing does not.

---

## 4. Startup Sequence

1. Power-on reset.
2. Hardware initialization (clocks, GPIO, timers, UART peripherals).
3. RTOS kernel initialization.
4. Entry into `usermain()`.
5. Creation and start of `ultrasonic_task`, `rf_task1`, and
   `motor_task`, in that order.
6. Transition to steady-state task execution under RTOS scheduling.

---

## 5. Subsystem Descriptions

### 5.1 `rf_task1` — RSSI Reception and Distance Prediction

**Reception path.** Incoming UART bytes are captured via interrupt and
written into a ring buffer. This design prevents data loss in cases
where bytes arrive faster than the task's polling interval can
consume them. A UART error callback clears hardware error flags and
re-arms reception, preventing a single transmission error from
permanently halting the RSSI data path.

**Feature extraction.** A rolling window of the five most recent RSSI
samples is maintained. From this window, five statistical features are
computed: mean, standard deviation, minimum, maximum, and median.

**Distance prediction.** The feature vector is passed to a trained
regression model (a shallow random forest, compiled to native C) to
produce a distance estimate.

**Anchor override.** The regression model exhibits reduced confidence
in the mid-range of its input domain, where it defaults to an
ambiguous estimate rather than a decisive one. To mitigate this, the
system applies threshold-based overrides at the extremes of the RSSI
range:

- RSSI values above a defined upper threshold are treated as
  indicating close proximity and are assigned a fixed minimum
  distance value.
- RSSI values below a defined lower threshold are treated as
  indicating the target is out of the model's reliable range and are
  assigned a fixed maximum distance value.
- RSSI values between these thresholds are passed through as the
  model's direct output.

These threshold values require calibration against measured
RSSI-versus-distance data for the deployed hardware and operating
environment.

### 5.2 `ultrasonic_task` — Obstacle Detection

Distance is measured using an HC-SR04 ultrasonic sensor via
time-of-flight. A hardware timer generates a periodic trigger pulse,
and an input-capture interrupt measures the duration of the returned
echo pulse, which is converted to a distance value.

Two reliability mechanisms are implemented:

1. **Pulse-width validation.** Captured pulses outside a defined valid
   range are discarded. This addresses electrical crosstalk between
   the trigger and echo signal lines, which are physically adjacent
   and can otherwise produce spurious near-zero-distance readings.
2. **Capture state recovery.** If an expected echo pulse is not
   received within an expected time window, the capture state machine
   is reset. This prevents desynchronization between trigger cycles
   and echo capture in cases where an echo is missed entirely.

### 5.3 `motor_task` — Navigation and Decision Logic

This task determines rover behavior based on sensor inputs, evaluated
in the following priority order on each execution cycle:

**Priority 1 — Obstacle avoidance.**
If ultrasonic data is unavailable or stale, the rover halts. If an
obstacle is detected within the defined stop distance, the rover
halts, performs a servo-actuated sensor sweep to identify the
clearest available heading, and executes a pivot turn toward that
heading. If RSSI-based distance estimation simultaneously indicates
arrival at the target, the detected object is treated as the target
itself rather than an obstacle, and the rover halts without
initiating avoidance behavior.

**Priority 2 — Target approach.**
If no obstacle is present, the rover proceeds toward the target using
RSSI-derived distance and direction information.

Because a single, effectively omnidirectional antenna cannot resolve
directional information from a single reading, direction is
determined by rotating the full chassis through a fixed number of
headings and sampling RSSI at each. This sweep is unidirectional: the
rover does not reverse rotation direction mid-sweep. A step counter
tracks cumulative rotation, and the sweep is considered complete when
this counter reaches the value corresponding to a full 360-degree
rotation. Because rotation is unidirectional, reaching this value
confirms that the rover has returned to its original orientation,
allowing it to face the strongest detected heading using only
additional forward rotation.

Approach behavior operates in two modes, determined by the predicted
distance to the target:

| Mode | Condition | Behavior |
|---|---|---|
| Coarse approach | Distance exceeds the model's reliable range | Drive toward the most recently identified heading at a fixed speed; re-evaluate heading periodically |
| Precision approach | Distance within the model's reliable range | Drive at a speed proportional to estimated distance, decelerating as the arrival threshold is approached |

**Arrival condition.** The rover halts when RSSI-based distance
estimation indicates arrival, provided the ultrasonic sensor also
reports a distance consistent with proximity to a physical object.
This cross-validation prevents an erroneous RSSI-based halt in the
absence of any physical confirmation.

**Motor speed constraints.** Maximum drive and turning speeds are
deliberately constrained below the mechanical maximum to ensure the
rover can decelerate to a stop within the defined obstacle margin.

---

## 6. Operational Sequence

1. System initialization completes; all tasks begin execution.
2. In the absence of valid RSSI data, the rover remains stationary.
3. Upon acquisition of valid RSSI data, an initial bearing sweep is
   performed to establish a heading toward the target.
4. The rover proceeds in coarse approach mode, periodically
   re-evaluating heading, until the target enters the model's
   reliable distance range.
5. The rover transitions to precision approach mode, decelerating as
   it approaches the arrival threshold.
6. If an obstacle is detected at any point, obstacle-avoidance logic
   takes precedence; upon resolution, a new bearing sweep is
   performed to re-establish heading, as the avoidance maneuver may
   have altered the rover's orientation.
7. The rover halts when both RSSI-based and ultrasonic-based distance
   measurements indicate arrival at the target.


