#pragma once

#include <stdbool.h>

/*
 * VESC-style smart reverse (mirrors use_smart_rev in bldc app_nunchuk.c):
 * holding brake past 92% while stopped switches to duty control and ramps
 * towards reverse, capped at SMART_REV_MAX_DUTY. It stays in duty control
 * until brake drops below 10%. Pure logic, no ESP-IDF, so it host-tests:
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_smart_reverse.c && /tmp/t
 */

#define SMART_REV_MAX_DUTY      0.07f  // VESC default smart_rev_max_duty
#define SMART_REV_RAMP_TIME_S   3.0f   // VESC default smart_rev_ramp_time
#define SMART_REV_ENTER_BRAKE   0.92f
#define SMART_REV_EXIT_BRAKE    0.10f

typedef struct {
    bool active;
    float duty;
} smart_reverse_t;

// brake: 0..1 brake input (0 when throttle is at or above neutral).
// stopped: every VESC is below the stopped ERPM threshold.
// Returns true while duty control owns the motors; s->duty is the duty to send.
static inline bool smart_reverse_step(smart_reverse_t *s, float brake, bool stopped, float dt) {
    if (!(s->active && brake > SMART_REV_EXIT_BRAKE) &&
        !(brake > SMART_REV_ENTER_BRAKE && stopped)) {
        s->active = false;
        s->duty = 0.0f;
        return false;
    }
    s->active = true;
    float goal = -SMART_REV_MAX_DUTY * brake;
    float step = SMART_REV_MAX_DUTY * dt / SMART_REV_RAMP_TIME_S;
    if (s->duty > goal + step)      s->duty -= step;
    else if (s->duty < goal - step) s->duty += step;
    else                            s->duty = goal;
    return true;
}
