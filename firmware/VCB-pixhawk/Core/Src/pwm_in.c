/*
 * pwm_in.c - see pwm_in.h
 */
#include "pwm_in.h"
#include "config.h"

typedef struct {
  TIM_HandleTypeDef* htim;
  /* written by the ISR only */
  volatile uint32_t glitch_count;     /* rise+fall pairs narrower than PWM_MIN_WIDTH_US, ignored */
  uint32_t rise_channel;   /* TIM_CHANNEL_x that captures the rising edge  */
  uint32_t fall_channel;   /* TIM_CHANNEL_x that captures the falling edge */
  uint32_t rise_of_flag;   /* TIM_FLAG_CCxOF of the rising capture */
  uint32_t fall_of_flag;

  /* written by the ISR only */
  volatile uint32_t rise_tick;
  volatile uint32_t orphan_fall_tick;  /* a falling capture with no candidate pending */
  volatile bool     have_orphan_fall;
  volatile uint32_t prev_rise_tick;
  volatile bool     have_rise;        /* a rising edge is waiting for its falling edge */
  volatile bool     have_prev_rise;   /* prev_rise_tick is meaningful */
  volatile bool     have_period;      /* period_us is meaningful */
  volatile uint32_t width_us;
  volatile uint32_t period_us;
  volatile uint32_t last_good_ms;
  volatile uint32_t last_edge_ms;
  volatile uint32_t ok_count;
  volatile uint32_t bad_count;
  volatile uint32_t ovr_count;
  volatile uint8_t  good_streak;
  volatile bool     valid;
} PwmChannel;

static PwmChannel ch[PWM_IN_CHANNELS];

static void channel_init(PwmChannel* c, TIM_HandleTypeDef* htim,
                         uint32_t rise_channel, uint32_t fall_channel,
                         uint32_t rise_of_flag, uint32_t fall_of_flag)
{
  c->htim = htim;
  c->rise_channel = rise_channel;
  c->fall_channel = fall_channel;
  c->rise_of_flag = rise_of_flag;
  c->fall_of_flag = fall_of_flag;
  c->rise_tick = c->prev_rise_tick = 0;
  c->have_rise = c->have_prev_rise = c->have_period = false;
  c->orphan_fall_tick = 0;
  c->have_orphan_fall = false;
  c->width_us = c->period_us = 0;
  c->last_good_ms = c->last_edge_ms = 0;
  c->ok_count = c->bad_count = c->ovr_count = 0;
  c->glitch_count = 0;
  c->good_streak = 0;
  c->valid = false;
}

void PwmIn_Start(TIM_HandleTypeDef* htim_ch1, TIM_HandleTypeDef* htim_ch2)
{
  /* Both timers: CH3 captures the rising edge, CH4 the falling edge (see
   * MX_TIM_IC_Common in main.cpp for why the order matters).
   * ch1: PA3 = TIM5 TI4 -> IC3 (indirect) rising, IC4 (direct) falling.
   * ch2: PA2 = TIM2 TI3 -> IC3 (direct) rising, IC4 (indirect) falling. */
  channel_init(&ch[0], htim_ch1, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_FLAG_CC3OF, TIM_FLAG_CC4OF);
  channel_init(&ch[1], htim_ch2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_FLAG_CC3OF, TIM_FLAG_CC4OF);

  for (int i = 0; i < PWM_IN_CHANNELS; i++) {
    if (HAL_TIM_IC_Start_IT(ch[i].htim, ch[i].rise_channel) != HAL_OK ||
        HAL_TIM_IC_Start_IT(ch[i].htim, ch[i].fall_channel) != HAL_OK) {
      Error_Handler();
    }
  }
}

/* A rejected pulse resets the qualification streak but does not by itself
 * drop `valid`: the last good width stays in force until no good pulse has
 * arrived for PWM_TIMEOUT_MS (see PwmIn_Snapshot). One glitch on the cable
 * must not open the contactors under load. */
static void mark_bad(PwmChannel* c)
{
  c->bad_count++;
  c->good_streak = 0;
}

/* A rising edge only records a candidate: nothing about the previous pulse
 * (period base, width) changes until a falling edge shows the candidate was a
 * real pulse. Glitches (rise and fall a few ticks apart, or the same tick)
 * are discarded whole in on_falling() and leave the period untouched. */
static void on_rising(PwmChannel* c, uint32_t tick, uint32_t now_ms)
{
  /* A rising edge while a candidate is already pending is not a bad pulse:
   * it happens when a glitch's falling capture was serviced before its rising
   * capture (channel order in the ISR), leaving a dangling candidate. A truly
   * missed falling capture shows up as an overrun instead. Take the newer edge. */
  if (c->have_orphan_fall) {
    c->have_orphan_fall = false;
    uint32_t d = tick - c->orphan_fall_tick;          /* either sign: the fall may carry the later tick */
    if (d < PWM_MIN_WIDTH_US || (0u - d) < PWM_MIN_WIDTH_US) {
      /* the other half of a glitch whose falling capture was serviced first:
       * not a candidate (otherwise the next glitch's fall would close it into
       * a fake pulse) */
      c->glitch_count++;
      c->last_edge_ms = now_ms;
      return;
    }
  }
  c->rise_tick = tick;
  c->have_rise = true;
  c->last_edge_ms = now_ms;
}

static void on_falling(PwmChannel* c, uint32_t tick, uint32_t now_ms)
{
  c->last_edge_ms = now_ms;
  if (!c->have_rise) {
    /* falling edge with no candidate: we started mid-pulse, or this is the
     * first half of a glitch pair serviced out of order (see on_rising) */
    c->orphan_fall_tick = tick;
    c->have_orphan_fall = true;
    return;
  }
  c->have_rise = false;
  uint32_t width = tick - c->rise_tick;

  if (width < PWM_MIN_WIDTH_US) {
    /* Narrower than any pulse the contract allows: a glitch. It is not a
     * pulse, so it does not count as bad, does not become the period base,
     * and does not touch the reported width. */
    c->glitch_count++;
    return;
  }

  /* A real pulse: the period is measured between accepted rising edges. */
  if (c->have_prev_rise) {
    uint32_t period = c->rise_tick - c->prev_rise_tick;   /* unsigned wrap is correct */
    c->period_us = period;
    /* After a gap longer than any accepted period (signal was off, or we
     * were not capturing), start over: this pulse is not judged. */
    c->have_period = (period <= PWM_MAX_PERIOD_US);
  }
  c->prev_rise_tick = c->rise_tick;
  c->have_prev_rise = true;
  c->width_us = width;

  if (!c->have_period) {
    return;   /* first pulse after boot or after a gap: no period to judge it by, count it neither way */
  }
  bool width_ok  = (width <= PWM_MAX_WIDTH_US);
  bool period_ok = (c->period_us >= PWM_MIN_PERIOD_US) && (c->period_us <= PWM_MAX_PERIOD_US) &&
                   (width < c->period_us);
  if (width_ok && period_ok) {
    c->ok_count++;
    if (c->good_streak < 255) c->good_streak++;
    if (c->good_streak >= PWM_GOOD_PULSES_TO_VALID) c->valid = true;
    c->last_good_ms = now_ms;
  } else {
    mark_bad(c);
  }
}

void PwmIn_CaptureCallback(TIM_HandleTypeDef* htim)
{
  uint32_t now_ms = HAL_GetTick();
  for (int i = 0; i < PWM_IN_CHANNELS; i++) {
    PwmChannel* c = &ch[i];
    if (htim != c->htim) continue;

    uint32_t active = 0;
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1: active = TIM_CHANNEL_1; break;
      case HAL_TIM_ACTIVE_CHANNEL_2: active = TIM_CHANNEL_2; break;
      case HAL_TIM_ACTIVE_CHANNEL_3: active = TIM_CHANNEL_3; break;
      case HAL_TIM_ACTIVE_CHANNEL_4: active = TIM_CHANNEL_4; break;
      default: return;
    }
    uint32_t tick = HAL_TIM_ReadCapturedValue(htim, active);

    if (active == c->rise_channel) {
      if (__HAL_TIM_GET_FLAG(htim, c->rise_of_flag)) { __HAL_TIM_CLEAR_FLAG(htim, c->rise_of_flag); c->ovr_count++; }
      on_rising(c, tick, now_ms);
    } else if (active == c->fall_channel) {
      if (__HAL_TIM_GET_FLAG(htim, c->fall_of_flag)) { __HAL_TIM_CLEAR_FLAG(htim, c->fall_of_flag); c->ovr_count++; }
      on_falling(c, tick, now_ms);
    }
    return;
  }
}

void PwmIn_Snapshot(uint8_t i, uint32_t now_ms, PwmInSample* out)
{
  if (i >= PWM_IN_CHANNELS) { out->valid = false; return; }
  PwmChannel* c = &ch[i];

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  out->width_us  = c->width_us;
  out->period_us = c->period_us;
  out->ok_count  = c->ok_count;
  out->bad_count = c->bad_count;
  out->ovr_count = c->ovr_count;
  out->glitch_count = c->glitch_count;
  bool valid = c->valid;
  uint32_t last_good = c->last_good_ms;
  uint32_t last_edge = c->last_edge_ms;
  uint32_t ok_count  = c->ok_count;

  /* timeout: no good pulse, or no edge at all, for PWM_TIMEOUT_MS */
  bool timed_out = (ok_count == 0) ||
                   ((now_ms - last_good) > PWM_TIMEOUT_MS) ||
                   ((now_ms - last_edge) > PWM_TIMEOUT_MS);
  if (valid && timed_out) {
    /* force re-qualification (PWM_GOOD_PULSES_TO_VALID good pulses) */
    c->valid = false;
    c->good_streak = 0;
    valid = false;
  }
  if (!primask) __enable_irq();

  out->valid  = valid;
  out->age_ms = (ok_count == 0) ? 0xFFFFFFFFu : (now_ms - last_good);
}
