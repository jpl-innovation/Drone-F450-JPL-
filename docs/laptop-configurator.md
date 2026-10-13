# Configurator over the air (laptop side)

1. On the Pi: `~/fcbridge.sh`, wait for `bridge on tcp://100.74.229.8:5760`.
2. On the laptop (fish), keep this loop running. "Connection refused" lines only mean the bridge isn't up yet:

```
while true; sudo socat pty,link=/dev/ttyDRONE,raw,echo=0,mode=666 tcp:drone:5760; sleep 1; end
```

3. INAV Configurator: Manual Selection, `/dev/ttyDRONE`, 115200, Connect.
4. Saving or leaving the CLI reboots the FC; the loops reconnect by themselves.
5. If the FC stops answering MSP, power the whole drone off and on again. `sudo reboot` on the Pi does not reset the FC.
6. Ctrl+C the bridge when done so telemetry restarts.
