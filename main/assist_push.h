#pragma once

#include <stdbool.h>

/*
 * Assistive push (also known as endless mode): kick the board and the receiver
 * keeps it rolling at the speed you pushed to, bleeding that speed off slowly
 * so one kick coasts for a while. Another kick picks the new speed up.
 *
 * ASSIST ONLY EVER DRIVES, NEVER BRAKES. It commands forward current, tapered
 * down as the board reaches the target speed, and commands zero as soon as the
 * rider is faster than the target. This is how pedal assist on an e-bike works,
 * and it is the whole design:
 *
 *   - A speed controller (SET_RPM) is bidirectional. The moment the rider
 *     pushes above its setpoint it brakes them back down to it, which feels
 *     like the board fighting the push. Torque-only assist cannot do that.
 * Assist gives up whenever the board falls well behind its target: the rider is
 * foot-braking or dragging to a stop, and the target decays far too slowly to
 * follow that. Without this, assist keeps pushing forward for seconds while the
 * rider is trying to stop, which feels like the board resisting.
 *
 * Assist is also armed only once the board has stopped: any use of the remote
 * disarms it until the next standstill, so it can never act as cruise control
 * after a throttle release, nor drive forward while the rider is braking.
 *
 *   - It also removes the overshoot ratchet: with a speed loop, adopting the
 *     measured speed as the new target makes the board accelerate itself. Here
 *     the target is only ever raised while assist is commanding zero current,
 *     i.e. while the board is coasting, so a rise can only have come from the
 *     rider's foot or from gravity — never from assist itself.
 *
 * The caller converts the speed window and decay rate into ERPM from the motor
 * config and sends the returned current with SET_CURRENT_REL. Pure logic, no
 * ESP-IDF, so it host-tests:
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_assist_push.c && /tmp/t
 */

// Defaults for the rider-tunable parameters (set from the remote, see throttle.c).
#define ASSIST_DECAY_RPM_S    30.0f  // motor rpm/s bled off the target speed
#define ASSIST_MAX_CURRENT    0.15f  // ceiling on assist current, fraction of l_current_max
#define ASSIST_DECAY_MIN      10.0f
#define ASSIST_DECAY_MAX      120.0f
#define ASSIST_CURRENT_MIN    0.05f
#define ASSIST_CURRENT_MAX    0.50f
#define ASSIST_CURRENT_SLEW   1.0f   // max change in that fraction per second
#define ASSIST_RISE_ERPM      50.0f  // per-sample rise that reads as the rider pushing
#define ASSIST_RISE_SAMPLES   3      // consecutive rising samples before assist takes it
#define ASSIST_GIVEUP_FULLS   1.5f   // falling this many full_erpm behind means the rider is slowing
#define ASSIST_GIVEUP_SAMPLES 15     // ~0.3 s behind before assist lets go

// Rider-tunable parameters, clamped to the limits above by the caller.
typedef struct {
    float max_current;  // fraction of l_current_max
    float decay_rpm_s;  // motor rpm/s
} assist_push_params_t;

typedef struct {
    bool engaged;
    bool armed;        // board has come to rest since the rider last used the remote
    int behind_count;  // consecutive samples the board has been far behind target
    float target;      // ERPM assist is trying to sustain
    float current;     // assist current now, 0..max_current
    float last_erpm;
    int rise_count;
} assist_push_t;

/*
 * erpm:         signed ERPM of the fastest motor (forward positive).
 * rider_input:  true when the throttle byte is off neutral — cancels assist.
 * stopped:      true when every motor is essentially at rest — re-arms assist.
 * min_erpm:     speed a push must reach before assist will take it.
 * max_erpm:     above this assist lets go entirely.
 * release_erpm: target winds down to here, then assist releases.
 * full_erpm:    speed shortfall at which assist commands its full current.
 * max_current:  ceiling on assist current, fraction of l_current_max.
 * decay_erpm_s: the rider's decay rate converted to ERPM/s.
 *
 * Returns true while assist is engaged; s->current is the forward current to
 * send, which is 0 whenever the rider is already at or above the target.
 */
static inline bool assist_push_step(assist_push_t *s, float erpm, bool rider_input,
                                    bool stopped, float min_erpm, float max_erpm,
                                    float release_erpm, float full_erpm,
                                    float max_current, float decay_erpm_s, float dt) {
    float prev = s->last_erpm;
    s->last_erpm = erpm;

    // Using the remote disarms assist: it may not engage again until the board
    // has come to rest. Without this, releasing the throttle mid-roll would let
    // assist latch that speed and hold it — cruise control nobody asked for —
    // and ERPM ripple under braking could read as a push and drive the board
    // forward while the rider is trying to stop.
    if (rider_input) {
        s->armed = false;
    } else if (stopped) {
        s->armed = true;
    }

    // Rider on the controls, rolling backwards, or past the cap: let go.
    if (rider_input || erpm <= 0.0f || erpm > max_erpm) {
        s->engaged = false;
        s->target = 0.0f;
        s->current = 0.0f;
        s->rise_count = 0;
        return false;
    }

    bool rising = erpm > prev + ASSIST_RISE_ERPM;
    s->rise_count = rising ? s->rise_count + 1 : 0;

    if (!s->engaged) {
        // Speed climbing on its own through the window: the rider is pushing.
        // Coasting down after a throttle release is not a push, so a rise is
        // required rather than just being in the window.
        if (!s->armed || erpm < min_erpm || prev <= 0.0f ||
            s->rise_count < ASSIST_RISE_SAMPLES) {
            return false;
        }
        s->engaged = true;
        s->target = erpm;
        s->current = 0.0f;
        s->behind_count = 0;
        return true;
    }

    // Raise the target only for a rise the rider caused: assist must be
    // commanding nothing AND the speed must still be climbing. Coasting above
    // the target (assist's own overshoot settling) must not move it, or the
    // board ratchets itself faster and faster.
    if (s->current <= 0.0f && s->rise_count >= ASSIST_RISE_SAMPLES &&
        erpm > s->target) {
        s->target = erpm;
    }

    s->target -= decay_erpm_s * dt;

    // The rider is slowing the board down faster than the target decays, or is
    // already at a crawl. Either way, stop pushing.
    bool far_behind = erpm < s->target - ASSIST_GIVEUP_FULLS * full_erpm;
    s->behind_count = far_behind ? s->behind_count + 1 : 0;
    if (s->target < release_erpm || erpm < release_erpm ||
        s->behind_count >= ASSIST_GIVEUP_SAMPLES) {
        s->engaged = false;
        s->target = 0.0f;
        s->current = 0.0f;
        s->behind_count = 0;
        return false;
    }

    // Proportional, forward-only: full current at full_erpm short of target,
    // tapering to zero as the board reaches it. Never negative, so a rider
    // pushing faster than the target meets no resistance at all.
    float shortfall = s->target - erpm;
    float wanted = 0.0f;
    if (shortfall > 0.0f) {
        wanted = shortfall / full_erpm * max_current;
        if (wanted > max_current) {
            wanted = max_current;
        }
    }

    // Slew-limit so assist fades in and out instead of stepping.
    float step = ASSIST_CURRENT_SLEW * dt;
    if (wanted > s->current + step)      s->current += step;
    else if (wanted < s->current - step) s->current -= step;
    else                                 s->current = wanted;
    if (s->current < 0.0f) {
        s->current = 0.0f;
    }
    return true;
}
