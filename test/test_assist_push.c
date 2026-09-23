/*
 * Host tests for assistive push. The properties that matter, in order:
 *   - it never resists the rider (current is never negative, and is zero
 *     whenever the rider is faster than the target),
 *   - it never accelerates the board past the speed that was pushed to,
 *   - it never engages under the rider's own throttle or brake,
 *   - it always winds down to a release.
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_assist_push.c && /tmp/t
 */
#include "../main/assist_push.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.02f  // receiver throttle task runs at 50 Hz

/* A 10-pole motor on typical esk8 gearing. */
#define ERPM_PER_KMH 400.0f
#define MIN_ERPM (5.0f * ERPM_PER_KMH)
#define MAX_ERPM (20.0f * ERPM_PER_KMH)
#define RELEASE_ERPM (2.0f * ERPM_PER_KMH)
#define FULL_ERPM (2.0f * ERPM_PER_KMH)
#define DECAY (ASSIST_DECAY_RPM_S * 5.0f)  // poles/2 = 5 → ERPM/s

#define STOPPED_ERPM 300.0f

static bool step_raw(assist_push_t *s, float erpm, bool rider_input) {
    return assist_push_step(s, erpm, rider_input, fabsf(erpm) < STOPPED_ERPM, MIN_ERPM,
                            MAX_ERPM, RELEASE_ERPM, FULL_ERPM, ASSIST_MAX_CURRENT, DECAY, DT);
}

/* Most tests start from a board at rest, which is what arms assist. */
static bool step(assist_push_t *s, float erpm, bool rider_input) {
    if (!s->armed && !s->engaged) {
        step_raw(s, 0.0f, false);  // standstill: arm
        s->last_erpm = erpm;       // don't let the jump read as a push
    }
    return step_raw(s, erpm, rider_input);
}

/* Kick the board up to `target`; assist engages as the speed climbs. */
static bool push_to(assist_push_t *s, float target) {
    bool on = false;
    for (float e = MIN_ERPM; e < target; e += ASSIST_RISE_ERPM * 2.0f) {
        on = step(s, e, false);
    }
    return step(s, target, false) || on;
}

static void test_never_engages_under_rider_input(void) {
    assist_push_t s = {0};
    for (float e = MIN_ERPM; e < MAX_ERPM; e += ASSIST_RISE_ERPM * 2.0f) {
        assert(!step(&s, e, true));  // throttle or brake held: never assist
    }
    assert(s.current == 0.0f);
}

static void test_window_and_direction(void) {
    assist_push_t s = {0};
    assert(!step(&s, MIN_ERPM - 500.0f, false));   // below walking pace
    assert(!step(&s, MIN_ERPM - 400.0f, false));
    assert(!step(&s, MAX_ERPM + 1000.0f, false));  // already past the cap
    assert(!step(&s, -4000.0f, false));            // rolling backwards
    assert(!step(&s, 4000.0f, false));             // direction flip is not a push
    assert(!step(&s, 4000.0f, false));             // steady coast is not a push
    assert(!step(&s, 4000.0f, false));
}

static void test_never_resists_the_rider(void) {
    /* The complaint this guards: with speed control the board braked whenever
     * the rider pushed faster than the target. Assist must go to zero current
     * and stay there while the rider is ahead of it. */
    assist_push_t s = {0};
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));

    float erpm = 10.0f * ERPM_PER_KMH;
    for (int i = 0; i < 200; i++) {
        erpm += 20.0f;  // rider keeps pushing, pulling ahead of the target
        assert(step(&s, erpm, false));
        assert(s.current >= 0.0f);   // never a braking command
        assert(s.current == 0.0f);   // and no drive either: pure free glide
        assert(s.target <= erpm + 1e-3f);  // target never gets ahead of the rider
    }
}

static void test_assist_overshoot_never_raises_target(void) {
    /* The runaway this guards: assist pulls, overshoots its own target, adopts
     * the overshoot, pulls harder. While assist is driving, a rise must never
     * move the target — only a coasting board may raise it. */
    assist_push_t s = {0};
    float pushed = 10.0f * ERPM_PER_KMH;
    assert(push_to(&s, pushed));

    /* Hold below target so assist ramps up real current. */
    for (int i = 0; i < 100; i++) step(&s, s.target - ERPM_PER_KMH, false);
    assert(s.current > 0.0f);

    float peak = s.target;
    for (int i = 0; i < 100; i++) {
        assert(step(&s, s.target + 0.3f * ERPM_PER_KMH, false));  // overshoot
        assert(s.target <= peak + 1e-3f);                         // never ratchets
        assert(s.current <= ASSIST_MAX_CURRENT + 1e-6f);
    }
    assert(s.target < peak);  // still decaying
}

static void test_current_tapers_with_shortfall(void) {
    assist_push_t s = {0};
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));

    /* Sagging behind the target, but not so far that the rider is clearly
     * stopping (that is the give-up rule): assist ramps to its ceiling. */
    for (int i = 0; i < 200; i++) {
        assert(step_raw(&s, s.target - 2.5f * ERPM_PER_KMH, false));
    }
    assert(fabsf(s.current - ASSIST_MAX_CURRENT) < 1e-3f);

    /* Back at target: assist fades away again. */
    for (int i = 0; i < 200; i++) step(&s, s.target, false);
    assert(s.current < 0.02f);
}

static void test_slew_limited(void) {
    assist_push_t s = {0};
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));
    float prev = s.current;
    for (int i = 0; i < 50; i++) {
        step(&s, 5.0f * ERPM_PER_KMH, false);  // big shortfall, wants full current
        assert(s.current - prev <= ASSIST_CURRENT_SLEW * DT + 1e-6f);
        prev = s.current;
    }
}

static void test_releases_near_standstill(void) {
    assist_push_t s = {0};
    assert(push_to(&s, 6.0f * ERPM_PER_KMH));
    bool released = false;
    for (int i = 0; i < 10000 && !released; i++) {
        released = !step(&s, s.target, false);
    }
    assert(released);            // never holds on forever
    assert(!s.engaged);
    assert(s.current == 0.0f);
}

static void test_lets_go_above_cap(void) {
    assist_push_t s = {0};
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));
    assert(!step(&s, MAX_ERPM + 500.0f, false));  // downhill past the cap
    assert(!s.engaged);
    assert(s.current == 0.0f);
}

static void test_brake_cancels_immediately(void) {
    assist_push_t s = {0};
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));
    assert(!step(&s, 10.0f * ERPM_PER_KMH, true));
    assert(!s.engaged);
    assert(s.current == 0.0f);
    assert(!step(&s, 10.0f * ERPM_PER_KMH, false));  // needs a fresh push
}

static void test_braking_never_drives_the_board(void) {
    /* The field bug: braking hard from ~11 km/h, ERPM ripple on the way down
     * read as a push and assist drove the board forward instead of letting it
     * stop. Assist must stay out of it until the board is at rest. */
    assist_push_t s = {0};
    assert(push_to(&s, 11.0f * ERPM_PER_KMH));

    float erpm = 11.0f * ERPM_PER_KMH;
    while (erpm > 1.0f * ERPM_PER_KMH) {
        erpm -= 60.0f;                                  // braking
        float ripple = (erpm > 5000.0f) ? 90.0f : 0.0f; // noisy ERPM on the way down
        assert(!step_raw(&s, erpm + ripple, true));     // brake held: never assist
        assert(s.current == 0.0f);
    }
    /* Brake released just above standstill: still must not engage or drive. */
    for (int i = 0; i < 100; i++) {
        erpm += 70.0f;  // ripple that would previously have read as a push
        assert(!step_raw(&s, erpm, false));
        assert(s.current == 0.0f);
    }
}

static void test_throttle_release_is_not_cruise_control(void) {
    /* Riding on the remote, then letting go: assist must not latch that speed.
     * It stays disarmed until the board has actually stopped. */
    assist_push_t s = {0};
    float erpm = 15.0f * ERPM_PER_KMH;
    for (int i = 0; i < 50; i++) assert(!step_raw(&s, erpm, true));  // on the throttle

    for (int i = 0; i < 500; i++) {   // throttle released, board coasting, some ripple
        erpm += (i % 2) ? 80.0f : -100.0f;
        assert(!step_raw(&s, erpm, false));
        assert(s.current == 0.0f);
    }

    /* Come to a stop — now assist is armed and a fresh push engages it. */
    step_raw(&s, 0.0f, false);
    assert(s.armed);
    assert(push_to(&s, 10.0f * ERPM_PER_KMH));
}

static void test_gives_up_when_rider_slows_the_board(void) {
    /* The field complaint: foot-braking to a stop, assist kept pushing forward
     * for seconds because the target decays far slower than the rider stops.
     * Falling behind the target must make assist let go quickly. */
    assist_push_t s = {0};
    assert(push_to(&s, 11.0f * ERPM_PER_KMH));

    float erpm = 11.0f * ERPM_PER_KMH;
    int samples = 0;
    bool released = false;
    while (samples < 500 && !released) {
        erpm -= 60.0f;  // rider dragging the board down, ~1.5 km/h per second
        released = !step_raw(&s, erpm, false);
        samples++;
    }
    assert(released);
    assert(s.current == 0.0f);
    /* Well under a second of assist, not the ~20 s the decay alone would take. */
    assert(samples * DT < 1.0f);
}

static void test_still_assists_a_mild_shortfall(void) {
    /* A small sag — rolling resistance, a gentle rise — is exactly what assist
     * is for, and must not trip the give-up rule. */
    assist_push_t s = {0};
    assert(push_to(&s, 11.0f * ERPM_PER_KMH));
    for (int i = 0; i < 200; i++) {
        assert(step_raw(&s, s.target - 0.5f * ERPM_PER_KMH, false));
    }
    assert(s.current > 0.0f);
}

int main(void) {
    test_never_engages_under_rider_input();
    test_window_and_direction();
    test_never_resists_the_rider();
    test_assist_overshoot_never_raises_target();
    test_current_tapers_with_shortfall();
    test_slew_limited();
    test_releases_near_standstill();
    test_lets_go_above_cap();
    test_brake_cancels_immediately();
    test_braking_never_drives_the_board();
    test_throttle_release_is_not_cruise_control();
    test_gives_up_when_rider_slows_the_board();
    test_still_assists_a_mild_shortfall();
    printf("assist push tests passed\n");
    return 0;
}
