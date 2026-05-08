#pragma once

#include <stdbool.h>
#include <stdint.h>

// Reasons a failsafe can be triggered — logged and queryable for diagnostics
typedef enum {
    FAILSAFE_REASON_THROTTLE_TIMEOUT = 0,  // Remote link lost
    FAILSAFE_REASON_CAN_FAULT,             // CAN bus hardware fault
    FAILSAFE_REASON_VESC_DROPOUT,          // Previously-active VESC stopped responding
    FAILSAFE_REASON_MANUAL,                // Explicitly requested (test/debug)
} failsafe_reason_t;

void failsafe_init(void);

// Trigger the failsafe from any context (ISR-safe via task notification).
// Idempotent — calling while already active is harmless.
void failsafe_trigger(failsafe_reason_t reason);

bool failsafe_is_active(void);

// Attempt to clear the failsafe.  Returns true only when:
//   - All VESCs are responding, AND
//   - Max |ERPM| across all VESCs is below the stopped threshold.
// Intended to be called when a valid throttle packet is received.
bool failsafe_try_clear(void);
