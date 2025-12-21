# BLE Data Protocol Reference

## Service UUID: 0xABF0
## Characteristic UUID: 0xABF2

## Combined Telemetry Data Packet
Total size: 60 bytes (Big-endian format)

| Parameter | Bytes | Position | Type | Scale | Units |
|-----------|--------|-----------|------|---------|-------|
| temp_mos | 2 | 0-1 | int16_t | ÷100 | °C |
| temp_motor | 2 | 2-3 | int16_t | ÷100 | °C |
| current_motor | 2 | 4-5 | int16_t | ÷100 | A |
| current_in | 2 | 6-7 | int16_t | ÷100 | A |
| rpm | 4 | 8-11 | int32_t | none | RPM |
| voltage | 2 | 12-13 | int16_t | ÷100 | V |
| bms_total_voltage | 2 | 14-15 | int16_t | ÷100 | V |
| bms_current | 2 | 16-17 | int16_t | ÷100 | A |
| remaining_capacity | 2 | 18-19 | int16_t | ÷100 | Ah |
| nominal_capacity | 2 | 20-21 | int16_t | ÷100 | Ah |
| num_cells | 1 | 22 | uint8_t | none | count |
| cell_voltages[16] | 32 | 23-54 | int16_t[] | ÷1000 | V |
| motor_poles | 1 | 55 | uint8_t | none | count |
| gear_ratio | 2 | 56-57 | uint16_t | ÷1000 | ratio |
| wheel_diameter | 2 | 58-59 | uint16_t | ÷1000 | meters |

### Notes
- `gear_ratio` and `wheel_diameter` come from `COMM_GET_MCCONF_TEMP` (ID 91) on the VESC.
- The receiver requests `COMM_GET_MCCONF_TEMP` on BLE connect and packs the three fields above.
- Other fields in `COMM_GET_MCCONF_TEMP` (current scales, ERPM limits, duty limits, and watt/current mins/maxes) are parsed but intentionally not forwarded over BLE; they are only used to keep packet offsets aligned.

