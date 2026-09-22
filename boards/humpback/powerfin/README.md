# Powerfin battery voltage

`board_adc` discovers the Linux IIO device named `battery-voltage` and reads
`in_voltage0_raw`, `in_voltage0_scale`, and optional `in_voltage0_offset`.
The device-tree `voltage-divider` must already include the hardware divider.
The IIO device number is not hardcoded. Scale and offset are loaded at driver
startup; restart the driver after changing them.

The adapter reports battery-side microvolts on PX4 ADC channel 0, with
`v_ref = 1` and `resolution = 1000000`. Thus the existing battery module
converts the report back to volts without applying the divider twice.
Failed samples are marked as unavailable channels, not voltage readings.
No current sensor is provided by this adapter.

`posix-configs/powerfin/px4_mc.config` starts `board_adc` and `battery_status`
and sets `BAT1_SOURCE=0`, `BAT1_V_CHANNEL=0`, and `BAT1_V_DIV=1`, replacing
the previous disabled source and legacy divider setting. Configure battery
cell count and voltage thresholds for the battery being used.

Build with `make humpback_powerfin_default`. Deploy both the resulting PX4
binary and updated Powerfin startup script using the normal board workflow.
In the PX4 console, verify:

```sh
board_adc status
listener adc_report
listener battery_status
```

Compare `battery_status.voltage_v` with `(raw + offset) * scale / 1000`
and a voltmeter. A zero raw reading is a valid disconnected/zero-voltage
measurement, not proof that the battery wiring works.
