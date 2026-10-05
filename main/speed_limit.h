#pragma once
#include <stdbool.h>

/*
 * Speed limit governor: PID on ERPM error, ceilinging current_rel as the
 * cap approaches. One shared computation from the dominant motor's ERPM,
 * broadcast to every motor identically (current_rel is a fraction of each
 * VESC's own current limit, so two motors can legitimately drift apart in
 * real speed under equal current - accepted trade for a synchronized,
 * jerk-free cut instead of a per-motor one).
 *
 * ponytail: the integral is primed from max_current (not the slewed
 * output) every sample outside the handoff window, so entering it starts
 * already near the right answer instead of learning from zero - the window
 * is only ~3 km/h and gets crossed in a fraction of a second under real
 * acceleration, too fast for a cold-started integral to converge in. Don't
 * "simplify" this back to priming from the output or the cut-to-neutral
 * jerk comes back.
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_speed_limit.c && /tmp/t
 */

#define SPEED_LIMIT_FULL_KMH 3.0f     // this far below the cap, no restriction at all
#define SPEED_LIMIT_CURRENT_SLEW 2.0f // max change in commanded current fraction per second

// Gains on normalized error (shortfall/full_erpm: 1.0 at the window edge,
// 0 at the cap) - dimensionless, portable across gearing/wheel setups.
#define SPEED_LIMIT_KP 1.0f
#define SPEED_LIMIT_KI 1.5f // per second
#define SPEED_LIMIT_KD 0.05f

typedef struct {
  float current; // current_rel commanded last step, post-slew
  float integral;
  float prev_error;
  bool have_prev;
} speed_limit_t;

// erpm/cap_erpm/full_erpm: motor ERPM, the cap, and SPEED_LIMIT_FULL_KMH in
// ERPM. max_current: rider's own current_rel request - a ceiling, never
// exceeded, never boosted. dt: seconds since last call.
static inline float speed_limit_step(speed_limit_t *s, float erpm,
                                     float cap_erpm, float full_erpm,
                                     float max_current, float dt) {
  float error = (cap_erpm - erpm) / full_erpm; // 1.0 at window edge, 0 at cap

  float wanted;
  if (error >= 1.0f) {
    wanted = max_current;
    s->integral = max_current - SPEED_LIMIT_KP; // primed for the window - see file header
    if (s->integral < -max_current) s->integral = -max_current;
    if (s->integral > max_current) s->integral = max_current;
    s->have_prev = false;
  } else {
    float derivative = s->have_prev ? (error - s->prev_error) / dt : 0.0f;
    s->prev_error = error;
    s->have_prev = true;

    s->integral += error * SPEED_LIMIT_KI * dt;
    if (s->integral < -max_current) s->integral = -max_current;
    if (s->integral > max_current) s->integral = max_current;

    wanted = SPEED_LIMIT_KP * error + s->integral + SPEED_LIMIT_KD * derivative;
  }
  if (wanted > max_current) wanted = max_current;
  if (wanted < 0.0f) wanted = 0.0f; // never brakes

  float step = SPEED_LIMIT_CURRENT_SLEW * dt;
  if (wanted > s->current + step)      s->current += step;
  else if (wanted < s->current - step) s->current -= step;
  else                                 s->current = wanted;
  return s->current;
}
