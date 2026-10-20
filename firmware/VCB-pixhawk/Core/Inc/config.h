/*
 * config.h - every tunable of the VCB Pixhawk-PWM firmware in one place.
 *
 * Units: pulse widths in microseconds (us), thrust in permille (1000 = 100 %),
 * times in milliseconds (ms), voltages in volts.
 */
#ifndef VCB_CONFIG_H
#define VCB_CONFIG_H

#define FW_NAME     "VCB pixhawk-pwm reversible"
#define FW_VERSION  "0.3.0"

/* ---- Thrust limit -------------------------------------------------------
 * Hard cap applied to the command sent to the ePropulsion converter, in both
 * directions. 200 = 20 %, the same posture as the deployed bench build.
 * Raise it deliberately, after the bench and dock tests in README.md.
 */
#define THRUST_LIMIT_PERMILLE        200

/* ---- Direction change ----------------------------------------------------
 * When the sign of a channel's command flips, 0 is sent instead until this
 * long has passed since the last non-zero command of the old direction, so
 * the converter always sees a stop between forward and reverse. Whether the
 * ePropulsion converter needs that is unknown (README, known unknowns); it is
 * cheap insurance against it dropping the motor on a direct reversal, the
 * same reaction it is reported to have to an out-of-range value. A stick
 * that rests in the deadband for longer than this adds no delay.
 * 0 = disabled (reverse directly, as cttdev's RC version did). */
#define DIRECTION_CHANGE_STOP_MS     200

/* ---- ePropulsion converter ----------------------------------------------
 * Value byte sent for 100 % thrust. cttdev/VCB used 0x7F (the byte is 7 bits
 * wide). Whether the converter's own full scale is 127 or 100 is unverified;
 * at the default 20 % limit the byte is 25 either way (20 % or 25 % thrust). */
#define EPROP_FULL_SCALE_VALUE       127

/* ---- PWM input contract (per channel, bidirectional servo convention) ----
 *   1000 us = full reverse, 1500 us = stop, 2000 us = full forward
 *   raw permille = (width - PWM_NEUTRAL_US) * 1000 / PWM_HALF_SPAN_US
 */
#define PWM_NEUTRAL_US               1500
#define PWM_HALF_SPAN_US             500
/* Deadband around neutral, then a continuous ramp to full scale:
 *   |d| <= DEADBAND            -> 0
 *   |d| >  DEADBAND            -> (|d| - DEADBAND) * 1000 / (HALF_SPAN - DEADBAND)
 * so 1525 us = 0 and 2000 us = 1000, with no step at the deadband edge.
 * 25 us covers the timebase error: the MCU runs from the HSI RC oscillator
 * (+-1 % or so over temperature; the deployed build measured +0.35 %), so a
 * true 1500 us pulse can read 1485..1515 us. */
#define PWM_DEADBAND_US              25

/* A pulse is "good" when its width and period are inside these windows.
 * The width window is deliberately wider than the 1000..2000 us contract: a
 * width inside the window but past an end point is accepted and pinned to
 * full scale (width_to_permille). It is never rejected and never becomes an
 * out-of-range converter byte. The deployed build is reported to have lost
 * the motors exactly that way, when a commanded 1900 us read as 1905 and
 * fell outside its mapped range. ArduPilot can emit 800..2200 us (SERVOn
 * limits, DO_SET_SERVO, motor test); with the ~1 % HSI timebase error that
 * is ~792..2222 us as measured here, so the window keeps margin beyond it.
 * Outside the window the pulse counts as bad: the last good width stays in
 * force and the channel goes invalid (-> stop) after PWM_TIMEOUT_MS. */
#define PWM_MIN_WIDTH_US             700
#define PWM_MAX_WIDTH_US             2300
#define PWM_MIN_PERIOD_US            2000    /* 500 Hz */
#define PWM_MAX_PERIOD_US            30000   /* 33 Hz  */

#define PWM_GOOD_PULSES_TO_VALID     3       /* consecutive good pulses before valid */
#define PWM_TIMEOUT_MS               100     /* no good pulse or no edge for this long -> invalid */

/* ---- Arming ------------------------------------------------------------
 * Both channels valid and within ARM_NEUTRAL_WINDOW_US of neutral for
 * ARM_HOLD_MS, while the thruster state machine is IDLE -> armed.
 * Neutral is also required to start the contactor sequence and all the way
 * until RUN: if either stick leaves the window while armed but not yet in
 * RUN, the board disarms (and aborts a start-up in progress) instead of
 * closing the contactors onto a non-zero command.
 */
#define ARM_NEUTRAL_WINDOW_US        50
#define ARM_HOLD_MS                  1000

/* ---- Fault recovery ----------------------------------------------------
 * After a FAULT the contactors are open and the thrusters are commanded to
 * stop. If FAULT_RECOVER_HOLD_MS > 0 the board returns to IDLE once both
 * channels have been valid and neutral for that long, and the normal arming
 * sequence runs again. 0 = latched until reset (deployed bench behaviour).
 * Interlock faults (contactor, bus voltage) are retried at most
 * INTERLOCK_FAULT_MAX_RETRIES times per boot; command-loss faults (1) are
 * unlimited; a console 'stop' (9) stays latched until 'clear' or reset.
 */
#define FAULT_RECOVER_HOLD_MS        3000
#define INTERLOCK_FAULT_MAX_RETRIES  3
/* 1 = a contactor opening in RUN (fault 8: E-stop or TSMS pulled) stays
 * latched until reset, i.e. releasing the E-stop does not re-close the
 * contactors by itself. 0 = recover like the other faults. */
#define CONTACTOR_OPEN_FAULT_LATCHED 1

/* ---- Bench mode: the console as command source ---------------------------
 * 'bench on' on the console (UART4, the Jetson's /dev/vcb) makes 'bench R L'
 * lines the command source instead of the PWM inputs, so the Jetson can spin
 * the motors directly, without the Pixhawk, in the commissioning plan
 * (docs/VCB_COMMISSIONING.md stage 3). Only accepted while disarmed. The
 * same arming rule applies (zero command for ARM_HOLD_MS), the same start-up
 * sequence runs, and losing the command stream is fault 1 like losing PWM.
 * BENCH_LIMIT_PERMILLE caps every bench command, under THRUST_LIMIT_PERMILLE. */
#define BENCH_LIMIT_PERMILLE         100     /* 10 %: a typo cannot exceed this */
#define BENCH_TIMEOUT_MS             500     /* no 'bench R L' for this long -> command lost */
#define CONSOLE_RX_RING_SIZE         128     /* bytes, power of two */

/* ---- Thruster startup sequence (unchanged from cttdev/VCB) -------------- */
#define IDLE_MAX_BUS_VOLTAGE_TO_START 5.0f   /* bus must be below this before LSC closes */
#define CONTACTOR_CLOSE_TIMEOUT_MS   1000    /* LSI/HSI must confirm within this */
#define STARTUP_WAIT_TIME_MS         500     /* between LSC closed and HSC energised */
#define STARTUP_HSC_VOLTAGE_CHECK_MS 10      /* after HSC closed, before the bus check */
#define STARTUP_MIN_BUS_VOLTAGE      40.0f   /* bus must reach this to enter RUN */
#define RUN_MIN_BUS_VOLTAGE          35.0f   /* bus must stay above this in RUN */
#define RUN_CONTACTOR_OPEN_DEBOUNCE_MS 50    /* LSI/HSI reading "open" in RUN for this long -> FAULT */

/* 1 = refuse to start the contactor sequence unless the E-stop and TSMS
 * 24 V sense inputs both read present. 0 = only report them (deployed
 * behaviour; the wiring of those senses is not verified). */
#define REQUIRE_ESTOP_TSMS_TO_START  0

/* ---- Periods ------------------------------------------------------------- */
#define MAIN_LOOP_PERIOD_MS          5
#define THRUSTER_CMD_PERIOD_MS       15      /* ePropulsion command rate in RUN (cttdev: every 15 ms loop) */
#define STOP_CMD_PERIOD_MS           100     /* "stop" to the converter when not in RUN */
#define MCB_IDLE_PERIOD_MS           100     /* "idle" to the two MCBs */
#define PWM_TELEM_PERIOD_MS          50      /* pwm,... line */
#define STATUS_TELEM_PERIOD_MS       200     /* status,... line */
#define HEADER_PERIOD_MS             10000   /* mapping + header lines */
#define IDLE_NOTE_PERIOD_MS          5000    /* "armed but bus too high" reminder */

/* ---- Bus voltage ADC (unchanged from cttdev/VCB) ------------------------ */
#define ADC_REF_VOLTAGE              3.3f
#define ADC_HALF_VALUE               32768.0f    /* 16 bit differential */
#define BUS_VOLTAGE_SCALAR           28.2727f    /* (30 + 30 + 2.2) / 2.2 */

/* ---- UART ----------------------------------------------------------------- */
#define UART_TX_TIMEOUT_MS           50      /* bounded, instead of HAL_MAX_DELAY */
#define CONSOLE_TX_BUF_SIZE          256

#endif /* VCB_CONFIG_H */
