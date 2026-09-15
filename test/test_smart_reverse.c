/*
 * Host tests for smart reverse: it must never engage while moving, never jump
 * past the duty cap, and must hand control back when the brake is released.
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_smart_reverse.c && /tmp/t
 */
#include "../main/smart_reverse.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.02f  // receiver throttle task runs at 50 Hz

static void test_needs_full_brake_and_stopped(void) {
    smart_reverse_t s = {0};
    assert(!smart_reverse_step(&s, 0.5f, true, DT));   // partial brake: normal braking
    assert(!smart_reverse_step(&s, 1.0f, false, DT));  // moving: brake, never reverse
    assert(smart_reverse_step(&s, 0.95f, true, DT));
}

static void test_ramps_to_cap_over_ramp_time(void) {
    smart_reverse_t s = {0};
    float prev = 0.0f;
    int steps = (int)(SMART_REV_RAMP_TIME_S / DT);
    for (int i = 0; i < steps - 1; i++) {
        assert(smart_reverse_step(&s, 1.0f, true, DT));
        assert(s.duty < prev + 1e-6f);                 // only moves towards reverse
        assert(s.duty >= -SMART_REV_MAX_DUTY - 1e-6f); // never past the cap
        prev = s.duty;
    }
    assert(s.duty > -SMART_REV_MAX_DUTY);  // not there before the ramp time
    for (int i = 0; i < 5; i++) smart_reverse_step(&s, 1.0f, true, DT);
    assert(fabsf(s.duty + SMART_REV_MAX_DUTY) < 1e-5f);
}

static void test_holds_while_rolling_until_release(void) {
    smart_reverse_t s = {0};
    smart_reverse_step(&s, 1.0f, true, DT);
    // Once rolling backwards the board is no longer "stopped"; keep reversing.
    assert(smart_reverse_step(&s, 0.5f, false, DT));
    assert(s.duty > -SMART_REV_MAX_DUTY * 0.5f - 1e-6f);  // goal scales with brake
    assert(!smart_reverse_step(&s, 0.05f, false, DT));    // released: back to normal
    assert(s.duty == 0.0f);
    assert(!smart_reverse_step(&s, 0.5f, false, DT));     // must re-enter from scratch
}

int main(void) {
    test_needs_full_brake_and_stopped();
    test_ramps_to_cap_over_ramp_time();
    test_holds_while_rolling_until_release();
    printf("smart reverse tests passed\n");
    return 0;
}
