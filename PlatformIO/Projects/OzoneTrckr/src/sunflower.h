#pragma once
#include <Arduino.h>

// ----------------------------------------------------------------------------
// Two-axis LDR sun tracker for the OzoneTrckr mount.
//
// Four LDRs in a 2x2 grid behind a cross-shade; we steer two servos to equalise
// the quadrants so the mount points at the sun. Moves are proportional (quick
// when far off) but HARD-CAPPED so the heavy mount never gets a momentum kick,
// and sub-degree (writeMicroseconds) so motion is smooth. Servos stay attached
// after begin() — holding torque keeps the mount from sagging.
// ----------------------------------------------------------------------------

void sunflowerBegin(int hPin, int vPin, int ldrLT, int ldrRT, int ldrLD, int ldrRD);
void sunflowerFindSun(uint32_t timeoutMs);
