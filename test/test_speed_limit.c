/*
 * Host tests: well clear of the cap it never restricts; never exceeds the
 * rider's request; never goes negative; a FAST approach (whole handoff
 * window crossed in ~0.3s, like real hard acceleration) doesn't cut to
 * neutral at the cap - the priming this depends on (see speed_limit.h); a
 * sustained closed loop settles at a steady nonzero current, not oscillating.
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_speed_limit.c && /tmp/t
 */
#include "../main/speed_limit.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.02f // receiver throttle task runs at 50 Hz
#define ERPM_PER_KMH 400.0f // a 10-pole motor on typical esk8 gearing
#define CAP_KMH 20.0f
#define CAP_ERPM (CAP_KMH * ERPM_PER_KMH)
#define FULL_ERPM (SPEED_LIMIT_FULL_KMH * ERPM_PER_KMH)

static void expect_close(float got, float want, const char *what) {
  if (fabsf(got - want) > 0.001f) {
    fprintf(stderr, "%s: got %f, want %f\n", what, got, want);
    assert(0);
  }
}

int main(void) {
  // Well clear of the cap: no restriction at all, once the slew (which
  // always applies, even from a standing start) has had a moment to catch
  // up.
  speed_limit_t s1 = {0};
  float out1 = 0.0f;
  for (int i = 0; i < 100; i++) {
    out1 = speed_limit_step(&s1, 0.0f, CAP_ERPM, FULL_ERPM, 0.8f, DT);
  }
  expect_close(out1, 0.8f, "well clear, 0.8 requested, slewed in");

  // Never exceeds the rider's own request, never negative, even mid-window.
  speed_limit_t s2 = {0};
  for (int i = 0; i < 500; i++) {
    float out = speed_limit_step(&s2, CAP_ERPM - FULL_ERPM / 2.0f, CAP_ERPM,
                                 FULL_ERPM, 0.3f, DT);
    assert(out >= 0.0f && out <= 0.3f + 0.0001f);
  }

  // Fast approach: full throttle from well clear of the cap, at ~10 km/h/s,
  // straight through the 3 km/h window (~0.3s). Current at the cap must not
  // be near zero.
  speed_limit_t s_fast = {0};
  float current_at_cap = -1.0f;
  {
    float erpm = CAP_ERPM - 2.0f * FULL_ERPM; // start outside the window
    float accel_erpm_s = 10.0f * ERPM_PER_KMH;
    bool crossed = false;
    for (int i = 0; i < 500 && !crossed; i++) {
      float current = speed_limit_step(&s_fast, erpm, CAP_ERPM, FULL_ERPM, 1.0f, DT);
      if (erpm >= CAP_ERPM) {
        current_at_cap = current;
        crossed = true;
      }
      erpm += accel_erpm_s * DT;
    }
    assert(crossed);
    if (current_at_cap < 0.15f) {
      fprintf(stderr,
              "fast approach: current at the cap is %f - still cutting to neutral\n",
              current_at_cap);
      assert(0);
    }
  }

  // Closed-loop simulation: a simple plant where erpm accelerates when
  // commanded current exceeds EQUILIBRIUM and decelerates otherwise (a
  // stand-in for drag). Starting well below the cap at full throttle, it
  // should climb to the cap and then SETTLE - steady current, steady speed.
  const float EQUILIBRIUM = 0.35f;
  const float PLANT_GAIN = 3000.0f; // erpm/s per unit of (current - equilibrium)
  speed_limit_t s3 = {0};
  float erpm = 0.0f;
  float last_currents[50] = {0};
  float last_erpms[50] = {0};
  int n = 0;
  for (int i = 0; i < 4000; i++) { // 80 simulated seconds, plenty to settle
    float current = speed_limit_step(&s3, erpm, CAP_ERPM, FULL_ERPM, 1.0f, DT);
    erpm += (current - EQUILIBRIUM) * PLANT_GAIN * DT;
    if (erpm < 0.0f) erpm = 0.0f;
    last_currents[n % 50] = current;
    last_erpms[n % 50] = erpm;
    n++;
  }
  float min_c = 2.0f, max_c = -1.0f, min_e = 1e9f, max_e = -1e9f;
  for (int i = 0; i < 50; i++) {
    if (last_currents[i] < min_c) min_c = last_currents[i];
    if (last_currents[i] > max_c) max_c = last_currents[i];
    if (last_erpms[i] < min_e) min_e = last_erpms[i];
    if (last_erpms[i] > max_e) max_e = last_erpms[i];
  }
  if (max_c - min_c > 0.05f) {
    fprintf(stderr, "current still oscillating at the end: min=%f max=%f\n", min_c, max_c);
    assert(0);
  }
  if (min_c < EQUILIBRIUM - 0.1f) {
    fprintf(stderr, "settled current %f is nowhere near equilibrium %f - still cutting to neutral\n",
            min_c, EQUILIBRIUM);
    assert(0);
  }
  if (fabsf((min_e + max_e) / 2.0f - CAP_ERPM) > FULL_ERPM) {
    fprintf(stderr, "settled speed %f is not near the cap %f\n", (min_e + max_e) / 2.0f, CAP_ERPM);
    assert(0);
  }

  printf("speed_limit: all tests passed (fast-approach current at cap ~%.3f; settled current ~%.3f, erpm ~%.0f, cap %.0f)\n",
        current_at_cap, (min_c + max_c) / 2.0f, (min_e + max_e) / 2.0f, CAP_ERPM);
  return 0;
}
