/*
 * pwm_in.h - two servo-PWM inputs measured with timer input capture.
 *
 *   ch1 = PA3 (CONT connector pin 2) on TIM5: TI4 -> IC4 rising, IC3 falling
 *   ch2 = PA2 (CONT connector pin 1) on TIM2: TI3 -> IC3 rising, IC4 falling
 *
 * Both timers run at 1 MHz (1 tick = 1 us), 32-bit free running. The ISR
 * measures period (rising to rising) and width (rising to falling), checks
 * both against the windows in config.h and keeps per-channel counters. The
 * main loop takes a snapshot with PwmIn_Snapshot() and applies the timeout.
 */
#ifndef PWM_IN_H
#define PWM_IN_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_IN_CHANNELS 2

typedef struct {
  uint32_t width_us;      /* last measured pulse width (0 until measured) */
  uint32_t period_us;     /* last measured period (0 until measured) */
  bool     valid;         /* PWM_GOOD_PULSES_TO_VALID consecutive good pulses and not timed out */
  uint32_t ok_count;      /* good pulses since boot */
  uint32_t bad_count;     /* rejected pulses since boot */
  uint32_t ovr_count;     /* capture overruns since boot */
  uint32_t age_ms;        /* time since the last good pulse */
} PwmInSample;

/* Call once after HAL_TIM_IC_Init() of both timers. Starts the captures. */
void PwmIn_Start(TIM_HandleTypeDef* htim_ch1, TIM_HandleTypeDef* htim_ch2);

/* Snapshot of channel `ch` (0 = ch1, 1 = ch2) at HAL tick `now_ms`,
 * with the timeout applied. Safe to call from the main loop. */
void PwmIn_Snapshot(uint8_t ch, uint32_t now_ms, PwmInSample* out);

/* To be called from HAL_TIM_IC_CaptureCallback(). */
void PwmIn_CaptureCallback(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif /* PWM_IN_H */
