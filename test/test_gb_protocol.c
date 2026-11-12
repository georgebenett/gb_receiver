// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Host tests for the shared BLE protocol: a known telemetry byte vector
 * decodes to the expected fields (sign bits included), and the advertised
 * name match is exact.
 *
 *   cc -Wall -Wextra -o /tmp/t test/test_gb_protocol.c && /tmp/t
 */
#include "../main/gb_protocol.h"
#include <assert.h>
#include <stdio.h>

static void test_telemetry_vector(void) {
  uint8_t buf[66] = {0};
  buf[0] = 0xC4, buf[1] = 0x09;                     // temp_mos 25.00
  buf[8] = 0x18, buf[9] = 0xFC;                     // erpm -1000
  buf[10] = 0xFF, buf[11] = 0xFF;
  buf[12] = 0x10, buf[13] = 0x0E;                   // v_in 36.00
  buf[22] = 10;                                     // cells
  buf[23] = 0x68, buf[24] = 0x10;                   // cell 0: 4200 mV
  buf[55] = 14;                                     // poles
  buf[56] = 0x98, buf[57] = 0x08;                   // gear 2.2
  buf[58] = 0x73, buf[59] = 0x00;                   // wheel 115 mm
  buf[60] = 1;                                      // aux on
  buf[61] = 0x01, buf[62] = 0x00, buf[63] = 0x00, buf[64] = 0x80; // INT32_MIN+1

  gb_telemetry_t t;
  assert(!gb_telemetry_decode(buf, 64, &t));
  assert(gb_telemetry_decode(buf, 65, &t));
  assert(t.temp_mos == 2500);
  assert(t.erpm == -1000);
  assert(t.v_in == 3600);
  assert(t.bms_cells == 10 && t.cell_mv[0] == 4200);
  assert(t.motor_poles == 14 && t.gear_ratio_x1000 == 2200);
  assert(t.wheel_diameter_mm == 115 && t.aux_on == 1);
  assert(t.trip_km_x100 == INT32_MIN + 1);
  assert(gb_telemetry_decode(buf, sizeof(buf), &t)); // newer, longer frame

  assert(gb_rd_i32(&buf[8]) == -1000);
  assert(gb_rd_i16(&buf[8]) == -1000);
  assert(gb_rd_u32(&buf[61]) == 0x80000001u);
  uint8_t w[4];
  gb_wr_u32(w, 0xA1B2C3D4u);
  assert(gb_rd_u32(w) == 0xA1B2C3D4u);
}

static void test_name_match(void) {
  const uint8_t full[] = "GS-THUMB";
  assert(gb_adv_name_matches(full, 8, GB_RECEIVER_ADV_NAME));
  assert(!gb_adv_name_matches(full, 0, GB_RECEIVER_ADV_NAME));
  assert(!gb_adv_name_matches(full, 2, GB_RECEIVER_ADV_NAME)); // "GS"
  assert(!gb_adv_name_matches(NULL, 8, GB_RECEIVER_ADV_NAME));
  const uint8_t longer[] = "GS-THUMBX";
  assert(!gb_adv_name_matches(longer, 9, GB_RECEIVER_ADV_NAME));
  // What a receiver really advertises: the name padded with zero bytes.
  const uint8_t padded[] = {'G', 'S', '-', 'T', 'H', 'U', 'M', 'B', 0, 0, 0, 0, 0};
  assert(gb_adv_name_matches(padded, sizeof(padded), GB_RECEIVER_ADV_NAME));
  const uint8_t nuls[] = {0, 0, 0};
  assert(!gb_adv_name_matches(nuls, sizeof(nuls), GB_RECEIVER_ADV_NAME));
}

int main(void) {
  test_telemetry_vector();
  test_name_match();
  printf("gb_protocol: all checks passed\n");
  return 0;
}
