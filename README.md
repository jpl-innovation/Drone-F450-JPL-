# Drone-F450-JPL

F450 quadcopter build: an INAV flight controller (MicoAir743 V2) with a Raspberry Pi 4 companion computer. The Pi streams live video through MediaMTX over Tailscale and reads flight telemetry from the FC over MSP.

## drone_telemetry (reader v2)

A small C++17 program that reads telemetry from INAV over the FC's USB-C port using MSP (MultiWii Serial Protocol, v1 framing plus the INAV v2 messages). It prints one line per poll to stdout (the journal) and sends JSON datagrams to the dashboard on the Pi's loopback address.

```
roll   12.3  pitch   -4.5  yaw  270 | 11.9 V   2.50 A    50 mAh  rssi  900 | loop 1000 us  modes 0x00000001
```

Design rules:
- **Read-only.** It only sends an allow-list of MSP "get" requests (status, attitude, motors, RC, IMU, altitude, battery, box names). It never writes settings or sends motor or RC commands.
- **No 1200 baud.** Some flight controllers reboot into their bootloader at that speed, so it is refused.
- **Checksum-verified parser.** Frames with a bad checksum are dropped.
- **Loopback only.** The UDP feed (`--udp=127.0.0.1:14650`) refuses any non-loopback address, so nothing leaves the Pi from this program.
- **Self-paced.** The next request goes out when the FC answers the previous one, capped at `--max-req-hz` (default 120). Attitude gets half of all slots.

### Build and run (Raspberry Pi OS)

```
sudo apt install build-essential cmake
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/drone_telemetry /dev/serial/by-id/<your-FC-by-id-name> 115200 5
```

Arguments: `<serial-port> [baud=115200] [print_hz=5] [--udp=127.0.0.1:14650|off] [--max-req-hz=120]`. Over USB the baud rate is ignored.

Close INAV Configurator first: the FC's USB port serves one computer at a time.

## Dashboard

`dashboard/drone_dashboard.py` is a read-only ground station in one Python file (standard library only). It listens for the reader's datagrams on `127.0.0.1:14650` (it falls back to the journal line, so it also works with an older reader) and serves a page on port 8080: video, attitude HUD, battery, motors, RC channels, radio link, Pi health and an event log. It only answers GET requests and never touches the FC's serial port.

```
python3 dashboard/drone_dashboard.py            # http://<pi>:8080
```

Options: `--host 0.0.0.0 --port 8080 --udp-port 14650 --cells 3`.

The video comes from MediaMTX (WebRTC on port 8889). A page opened over `https` (for example through Tailscale Funnel) plays the HLS stream on port 8443 instead, because Funnel cannot carry WebRTC.

The page has **no login**. Expose it only on a private network such as Tailscale (the Pi's firewall should allow nothing else except SSH from the bench LAN), or knowingly share it with Funnel.

To change the page, edit `dashboard/src/index.html` or `dashboard/src/server.py` and run `python3 dashboard/src/build.py`.

Radio link: this receiver reports no signal quality (INAV's `rssi` is fixed), so the Radio link card can only show ON or LOST.

### Run at boot (systemd)

`deploy/drone-telemetry.service` and `deploy/drone-dashboard.service`. Edit the user name and the FC's `/dev/serial/by-id/` name first, then:

```
sudo cp deploy/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now drone-telemetry drone-dashboard
```

Do not add `After=multi-user.target` to a service that is `WantedBy=multi-user.target`: systemd finds an ordering cycle and silently drops the job at boot. Keep the journal in RAM (`Storage=volatile` in a journald drop-in) so 5 lines a second don't wear the SD card.

## Hardware

F450 frame, MicoAir743 V2 flight controller (INAV 9.1.0) with the MicoAir IO board, 4x A2212 1000KV motors, 4x BLHeli_S 30A ESCs, 1045 props, 3S 2200 mAh LiPo, Microzone MC6C radio with an MC7RE receiver, Raspberry Pi 4 (4 GB), Raspberry Pi Camera Module 3 (earlier: Logitech C920-C webcam).

## Status

Work in progress. Bench-tested on the real FC and a simulated one. Not flown yet.
