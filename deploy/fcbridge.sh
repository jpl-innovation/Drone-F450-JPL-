#!/bin/bash
# Wireless INAV Configurator bridge. Run on the Pi: ~/fcbridge.sh
# Pauses the telemetry reader (it owns the FC's USB port), exposes the FC on
# tcp://<Tailscale IP>:5760, and restarts telemetry when you press Ctrl+C.
# Reference version rebuilt from how the bridge behaves; edit FC to your by-id name.
FC=$(ls /dev/serial/by-id/*INAV* | head -n1)
IP=$(tailscale ip -4 | head -n1)
sudo systemctl stop drone-telemetry
trap 'sudo systemctl start drone-telemetry; echo "bridge stopped, telemetry back on"; exit 0' INT TERM
echo "bridge on tcp://$IP:5760"
sudo socat TCP-LISTEN:5760,bind="$IP",reuseaddr,fork "$FC",raw,echo=0,b115200
