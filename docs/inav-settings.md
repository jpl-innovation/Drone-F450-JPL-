# INAV 9.1.0 settings we changed (MicoAir743 V2)

Set from the Modes tab and the CLI, then `save`:

```
aux 0 0 0 1300 2100          # ARM on CH5 high
aux 1 1 0 900 2100           # ANGLE on CH5 (both modes share the switch)
set rx_min_usec = 950
set nav_disarm_on_landing = OFF
set nav_emerg_landing_speed = 200
```

Other values in use: DSHOT300, `control_profile 1`, SBUS receiver on UART6 (Ports tab: Serial RX).
If the receiver is dead after a reflash, check UART6 Serial RX there first.
Props-off bench note: with ANGLE active and no motion, the I-term winds up and motors read uneven (e.g. 80/35/35/8 %). That is normal without props.
Save your own `diff all` output next to this file after any change.
