# Drone-F450-JPL

F450 quadcopter build: an INAV flight controller (MicoAir743 V2) with a Raspberry Pi 4 companion computer. The Pi streams live video from a USB webcam through MediaMTX over Tailscale and reads flight telemetry from the FC over MSP.

## drone_telemetry

A small C++17 program that reads telemetry from INAV over the FC's USB-C port using MSP v1 (MultiWii Serial Protocol).

It polls 5 times a second and prints one line per poll:

```
roll   12.3  pitch   -4.5  yaw  270 | 11.9 V   2.50 A    50 mAh  rssi  900 | loop 1000 us  modes 0x00000001
```

Design rules:
- **Read-only.** It only sends an allow-list of MSP "get" requests (API version, status, attitude, analog). It never writes settings or sends motor or RC commands.
- **No 1200 baud.** Some flight controllers reboot into their bootloader at that speed, so it is refused.
- **Checksum-verified parser.** Frames with a bad checksum are dropped.
- **Nothing goes to the network.** Output is stdout only.

### Build and run (Raspberry Pi OS)

```
sudo apt install build-essential cmake
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/drone_telemetry /dev/serial/by-id/<your-FC-by-id-name> 115200
```

Arguments: `<serial-port> [baud=115200] [rate_hz=5]`. Over USB the baud rate is ignored.

Close INAV Configurator first: the FC's USB port serves one computer at a time.

### Run at boot (systemd)

Run it as a normal user in the `dialout` group with a unit file that sets `Restart=always`. Keep the journal in RAM (`Storage=volatile` in a journald drop-in) so 5 lines a second don't wear the SD card.

## Hardware

F450 frame, MicoAir743 V2 flight controller (INAV 9.1.0) with the MicoAir IO board, 4x A2212 1000KV motors, 4x BLHeli_S 30A ESCs, 1045 props, 3S 2200 mAh LiPo, Microzone MC6C radio with an MC7RE receiver, Raspberry Pi 4 (4 GB), Logitech C920-C webcam.

## Status

Work in progress. Bench-tested on the real FC and a simulated one. Not flown yet.
