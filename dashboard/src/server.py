#!/usr/bin/env python3
"""drone_dashboard.py v2 - read-only glass-cockpit dashboard for the F450 (JPL Innovation).

Data sources (in order of preference):
  1. drone_telemetry v2 sends JSON datagrams to 127.0.0.1:14650 (attitude ~50 Hz,
     battery/motors/RC/IMU/altitude/status ~8-10 Hz).
  2. If no datagram arrives for 2 s, it falls back to the old journal line (5 Hz, fewer fields),
     so the page still works with the v1 reader.
Pi health (CPU temperature, under-voltage, load, network) is read locally every 2 s.

Safety: it never opens the flight controller's serial port, never sends anything to the FC,
and answers GET requests only. The video comes straight from MediaMTX (port 8889) to the browser.

Usage on the Pi:   python3 drone_dashboard.py            -> http://<pi>:8080
Options:           --port 8080 --host 0.0.0.0 --udp-port 14650 --cells 3
                   --journal-cmd "journalctl -fu drone-telemetry -n 1 -o cat --no-pager"
"""
import argparse
import json
import os
import re
import shlex
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

INDEX_HTML = r'''__INDEX_HTML__'''

VERSION = '2.2'
cond = threading.Condition()
S = {
    'seq': 0, 'tele': None, 'tele_t': None, 'src': None, 'udp_t': None,
    'meta': None, 'pi': {}, 'armed': False, 'armed_since': None, 'armed_total': 0.0,
}
EVENTS = deque(maxlen=200)
EV = {'next': 1}
HIST_N = 600  # 120 s at 5 Hz
HIST = {k: deque(maxlen=HIST_N) for k in ('t', 'v', 'a', 'm1', 'm2', 'm3', 'm4', 'alt', 'lq')}
HIST_LAST = {'t': 0.0}
OPTS = {'cells': 3}


def add_event(level, msg):
    with cond:
        EVENTS.append({'id': EV['next'], 'ts': time.time(), 'level': level, 'msg': msg})
        EV['next'] += 1
        cond.notify_all()


def arm_index():
    meta = S.get('meta') or {}
    names = meta.get('boxnames') or []
    return names.index('ARM') if 'ARM' in names else 0


def mode_on(tele, name):
    names = (S.get('meta') or {}).get('boxnames') or []
    if name not in names:
        return False
    i = names.index(name)
    st = (tele or {}).get('status') or {}
    word = st.get('modes_lo', 0) if i < 32 else st.get('modes_hi', 0)
    return bool((word >> (i % 32)) & 1)


# ---- radio (RC) link: transmitter -> receiver -> FC
# INAV 9.1 facts this relies on:
#  * with rssi_source AUTO and no RSSI wire, 'rssi' is link quality from the receiver protocol
#    (SBUS: share of good frames per 200 ms; drops to 0 after 1 s without frames)
#  * arming flag bit 18 ARMING_DISABLED_RC_LINK = no RX data (only updated while disarmed)
#  * box FAILSAFE is active while the failsafe procedure runs (armed signal loss)
RC_LINK_FLAG = 1 << 18
LQ_WEAK = 70
RC = {'state': 'unknown', 'lq': None, 'seen_lq': False, 'losses': 0, 'last_loss': None,
      'lost_since': None, 'min_lq': None, 'weak': False}


def lq_pct(tele):
    r = (tele or {}).get('rssi')
    return None if r is None else int(round(min(max(r, 0), 1023) * 100 / 1023))


def rc_link_state(tele):
    lq = lq_pct(tele)
    if lq:
        RC['seen_lq'] = True
    st = (tele or {}).get('status') or {}
    rc = (tele or {}).get('rc') or []
    # this receiver, when the transmitter is off, keeps sending frames with a preset:
    # throttle 905 and every other channel 1505. A real throttle stick never goes below ~1000.
    preset = len(rc) >= 4 and 0 < rc[3] < 950
    if mode_on(tele, 'FAILSAFE'):
        return 'failsafe', lq
    if preset:
        return 'lost', 0
    if (st.get('arming') or 0) & RC_LINK_FLAG and not is_armed(tele):
        return 'lost', lq
    if RC['seen_lq'] and lq == 0:
        return 'lost', lq
    if not RC['seen_lq']:
        return 'nolq', lq
    return 'ok', lq


def rc_info(now):
    with cond:
        r = dict(RC)
    return {'state': r['state'], 'lq': r['lq'], 'losses': r['losses'], 'min_lq': r['min_lq'], 'weak': r['weak'],
            'since_loss': round(now - r['last_loss'], 1) if r['last_loss'] else None,
            'lost_for': round(now - r['lost_since'], 1) if r['lost_since'] else None}


def is_armed(tele):
    st = (tele or {}).get('status') or {}
    i = arm_index()
    word = st.get('modes_lo', 0) if i < 32 else st.get('modes_hi', 0)
    return bool((word >> (i % 32)) & 1)


def store_tele(tele, src):
    now = time.monotonic()
    with cond:
        S['tele'] = tele
        S['tele_t'] = now
        S['src'] = src
        S['seq'] += 1
        if now - HIST_LAST['t'] >= 0.2:  # history at 5 Hz
            HIST_LAST['t'] = now
            bat = tele.get('bat') or {}
            mot = tele.get('motors') or []
            HIST['t'].append(round(time.time(), 1))
            HIST['v'].append(bat.get('v'))
            HIST['a'].append(bat.get('a'))
            for i in range(4):
                HIST['m%d' % (i + 1)].append(mot[i] if i < len(mot) else None)
            HIST['alt'].append((tele.get('alt') or {}).get('m'))
            HIST['lq'].append(lq_pct(tele))
        cond.notify_all()


# ---------------------------------------------------------------- sources
def udp_listener(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))  # loopback only
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            msg = json.loads(data.decode('utf-8', 'replace'))
        except Exception:
            continue
        if msg.get('type') == 'meta':
            with cond:
                S['meta'] = msg
        elif msg.get('type') == 't':
            S['udp_t'] = time.monotonic()
            store_tele(msg, 'reader v2')


NUM = r'(-?\d+(?:\.\d+)?)'
PATS = {
    'roll': re.compile(r'roll\s+' + NUM), 'pitch': re.compile(r'pitch\s+' + NUM),
    'yaw': re.compile(r'yaw\s+' + NUM), 'volt': re.compile(NUM + r'\s*V\b'),
    'amp': re.compile(NUM + r'\s*A\b'), 'mah': re.compile(r'(\d+)\s*mAh'),
    'rssi': re.compile(r'rssi\s+(\d+)'), 'loop': re.compile(r'loop\s+(\d+)\s*us'),
    'modes': re.compile(r'modes\s+0x([0-9a-fA-F]+)'),
}


def parse_line(line):
    if 'roll' not in line or 'pitch' not in line or 'yaw' not in line:
        return None
    v = {}
    for k, rx in PATS.items():
        m = rx.search(line)
        if m:
            v[k] = m.group(1)
    if not all(k in v for k in ('roll', 'pitch', 'yaw', 'volt', 'amp')):
        return None
    modes = int(v.get('modes', '0'), 16)
    return {
        'type': 't', 'att': {'roll': float(v['roll']), 'pitch': float(v['pitch']), 'yaw': int(float(v['yaw'])), 'age': 0},
        'bat': {'v': float(v['volt']), 'a': float(v['amp']), 'mah': int(v.get('mah', 0)), 'src': 'journal', 'age': 0,
                'pct': -1, 'cells': 0, 'state': -1},
        'rssi': int(v.get('rssi', 0)),
        'status': {'cycle': int(v.get('loop', 0)), 'modes_lo': modes, 'modes_hi': 0, 'src': 'journal', 'age': 0},
        'motors': [], 'rc': [], 'rates': {'att': 5},
    }


def journal_reader(cmd):
    backoff = 1.0
    while True:
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            backoff = 1.0
            for line in proc.stdout:
                udp_t = S.get('udp_t')
                if udp_t is not None and time.monotonic() - udp_t < 2.0:
                    continue  # the fast feed is alive: ignore the journal
                rec = parse_line(line)
                if rec:
                    store_tele(rec, 'journal (5 Hz)')
            proc.wait()
        except Exception as exc:
            print('journal reader:', exc, file=sys.stderr, flush=True)
        time.sleep(backoff)
        backoff = min(backoff * 2, 10.0)


# ---------------------------------------------------------------- Pi health
def read(path, default=None):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return default


def pi_health_loop():
    prev = None
    while True:
        h = {}
        t = read('/sys/class/thermal/thermal_zone0/temp')
        h['temp'] = round(int(t) / 1000.0, 1) if t and t.strip().isdigit() else None
        try:
            out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=2).stdout
            m = re.search(r'0x([0-9a-fA-F]+)', out)
            h['throttled'] = int(m.group(1), 16) if m else None
        except Exception:
            h['throttled'] = None
        try:
            h['load'] = [round(x, 2) for x in os.getloadavg()]
        except Exception:
            h['load'] = None
        mi = read('/proc/meminfo', '')
        tot = re.search(r'MemTotal:\s+(\d+)', mi)
        av = re.search(r'MemAvailable:\s+(\d+)', mi)
        if tot and av:
            h['mem_used_pct'] = round(100 * (1 - int(av.group(1)) / int(tot.group(1))))
        up = read('/proc/uptime')
        h['uptime'] = int(float(up.split()[0])) if up else None
        st = read('/proc/stat', '')
        if st.startswith('cpu '):
            vals = [int(x) for x in st.split('\n', 1)[0].split()[1:]]
            idle, total = vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)
            if prev:
                dt, di = total - prev[0], idle - prev[1]
                h['cpu_pct'] = round(100 * (1 - di / dt)) if dt > 0 else None
            prev = (total, idle)
        nets = []
        wl = read('/proc/net/wireless', '')
        for name in sorted(os.listdir('/sys/class/net')) if os.path.isdir('/sys/class/net') else []:
            if name == 'lo':
                continue
            flags = read('/sys/class/net/%s/flags' % name, '0x0').strip()
            oper = (read('/sys/class/net/%s/operstate' % name, '') or '').strip()
            try:
                is_up = bool(int(flags, 16) & 1) and oper in ('up', 'unknown')
            except ValueError:
                is_up = False
            item = {'name': name, 'up': is_up}
            m = re.search(r'^\s*%s:\s+\S+\s+([\d.]+)\.?\s+(-?[\d.]+)' % re.escape(name), wl, re.M)
            if m:
                item['dbm'] = round(float(m.group(2)))
            nets.append(item)
        h['net'] = nets
        old = S.get('pi') or {}
        with cond:
            S['pi'] = h
        th = h.get('throttled')
        if th is not None and (th & 1) and not ((old.get('throttled') or 0) & 1):
            add_event('bad', 'Pi UNDER-VOLTAGE now: check the BEC / USB-C cable')
        if h.get('temp') and h['temp'] >= 75 and not ((old.get('temp') or 0) >= 75):
            add_event('warn', 'Pi CPU hot: %.0f °C' % h['temp'])
        time.sleep(2)


# ---------------------------------------------------------------- events
def monitor_loop():
    link_ok = None
    bat_level = 0  # 0 ok, 1 warn, 2 crit (per arm cycle)
    low_since = None
    weak_since = None
    while True:
        time.sleep(0.2)
        with cond:
            tele, t = S['tele'], S['tele_t']
        now = time.monotonic()
        alive = t is not None and now - t < 1.5
        if link_ok is None:
            if alive:
                add_event('info', 'Telemetry connected (%s)' % S.get('src'))
                link_ok = True
        elif alive != link_ok:
            add_event('info' if alive else 'bad', 'Telemetry restored' if alive else 'Telemetry LOST (no data for 1.5 s)')
            link_ok = alive
        if not alive or not tele:
            with cond:
                RC['state'] = 'unknown'
            continue
        armed = is_armed(tele)
        if armed != S['armed']:
            with cond:
                S['armed'] = armed
                if armed:
                    S['armed_since'] = now
                    bat_level = 0
                    RC['min_lq'] = None
                else:
                    dur = now - (S['armed_since'] or now)
                    S['armed_total'] += dur
                    S['armed_since'] = None
            if armed:
                add_event('warn', 'ARMED')
            else:
                add_event('info', 'Disarmed after %d:%02d' % (int(dur) // 60, int(dur) % 60))
        state, lq = rc_link_state(tele)
        lost = state in ('lost', 'failsafe')
        with cond:
            prev = RC['state']
            RC['state'], RC['lq'] = state, lq
            if lq is not None and state == 'ok':
                RC['min_lq'] = lq if RC['min_lq'] is None else min(RC['min_lq'], lq)
        if lost and prev not in ('lost', 'failsafe'):
            with cond:
                RC['losses'] += 1
                RC['last_loss'] = RC['lost_since'] = now
            add_event('bad', 'RADIO LINK LOST: receiver is not hearing the transmitter' +
                      (' (INAV failsafe running)' if state == 'failsafe' else ''))
        elif not lost and prev in ('lost', 'failsafe'):
            dur = now - (RC['lost_since'] or now)
            with cond:
                RC['lost_since'] = None
            add_event('info', 'Radio link back after %.1f s' % dur)
        if state == 'ok' and lq is not None and lq < LQ_WEAK:
            weak_since = weak_since or now
            if now - weak_since >= 1.0 and not RC['weak']:
                RC['weak'] = True
                add_event('warn', 'Radio signal weak: %d%% of frames arriving. Turn back.' % lq)
        else:
            weak_since = None
            if RC['weak'] and (state != 'ok' or (lq or 0) >= 85):
                RC['weak'] = False
        if state == 'failsafe' and prev == 'lost':
            add_event('bad', 'INAV failsafe running')
        bat = tele.get('bat') or {}
        v = bat.get('v') or 0
        cells = bat.get('cells') or OPTS['cells']
        if v > 1 and cells:
            per = v / cells
            level = 2 if per < 3.3 else (1 if per < 3.5 else 0)
            if level > bat_level:
                low_since = low_since or now
                if now - low_since >= 2.0:  # sustained, not a throttle punch
                    bat_level = level
                    add_event('bad' if level == 2 else 'warn',
                              ('Battery CRITICAL' if level == 2 else 'Battery low') + ': %.2f V (%.2f V/cell)' % (v, per))
            else:
                low_since = None


# ---------------------------------------------------------------- HTTP
def flight_info(now):
    with cond:
        armed, since, total = S['armed'], S['armed_since'], S['armed_total']
    cur = (now - since) if (armed and since) else 0.0
    return {'armed': armed, 'armed_for': round(cur, 1), 'total': round(total + cur, 1)}


def build_msg(include_slow, last_ev):
    now = time.monotonic()
    with cond:
        tele, t, src, seq = S['tele'], S['tele_t'], S['src'], S['seq']
        new_ev = [e for e in EVENTS if e['id'] > last_ev]
    msg = {'seq': seq, 'src': src, 'age': round(now - t, 2) if t else None, 'tele': tele,
           'flight': flight_info(now), 'rc': rc_info(now), 'server_time': time.time()}
    if include_slow:
        msg['pi'] = S.get('pi')
        msg['meta'] = S.get('meta')
        msg['ver'] = VERSION
    if new_ev:
        msg['events'] = new_ev
    return msg, (new_ev[-1]['id'] if new_ev else last_ev)


class Handler(BaseHTTPRequestHandler):
    server_version = 'drone-dashboard/' + VERSION
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def _head(self, code, ctype, length=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; "
                         "img-src 'self' data:; media-src 'self' blob:; connect-src 'self' http://*:8889 https://*:8889; frame-src https://*:8443")
        if length is not None:
            self.send_header('Content-Length', str(length))

    def _json(self, obj):
        body = json.dumps(obj, separators=(',', ':')).encode()
        self._head(200, 'application/json', len(body))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        path, q = u.path, parse_qs(u.query)
        if path in ('/', '/index.html'):
            body = INDEX_HTML.encode('utf-8')
            self._head(200, 'text/html; charset=utf-8', len(body))
            self.end_headers()
            self.wfile.write(body)
        elif path == '/api/history':
            with cond:
                self._json({k: list(v) for k, v in HIST.items()})
        elif path == '/api/events':
            with cond:
                self._json(list(EVENTS))
        elif path == '/api/state':
            self._json(build_msg(True, 10 ** 9)[0])
        elif path == '/healthz':
            self._head(200, 'text/plain', 3)
            self.end_headers()
            self.wfile.write(b'ok\n')
        elif path == '/events':
            try:
                hz = max(1.0, min(50.0, float(q.get('hz', ['25'])[0])))
            except ValueError:
                hz = 25.0
            period = 1.0 / hz
            self._head(200, 'text/event-stream')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            seen, last_ev, last_slow = -1, 0, 0.0
            try:
                while True:
                    t0 = time.monotonic()
                    with cond:
                        cond.wait_for(lambda: S['seq'] != seen or EV['next'] - 1 > last_ev, timeout=1.0)
                    slow = time.monotonic() - last_slow >= 1.0
                    msg, last_ev = build_msg(slow, last_ev)
                    if slow:
                        last_slow = time.monotonic()
                    seen = msg['seq']
                    self.wfile.write(('data: %s\n\n' % json.dumps(msg, separators=(',', ':'))).encode())
                    self.wfile.flush()
                    rest = period - (time.monotonic() - t0)
                    if rest > 0:
                        time.sleep(rest)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        else:
            body = b'not found\n'
            self._head(404, 'text/plain', len(body))
            self.end_headers()
            self.wfile.write(body)

    def _refuse(self):
        self._head(405, 'text/plain', 20)
        self.send_header('Allow', 'GET')
        self.end_headers()
        self.wfile.write(b'read-only dashboard\n')
    do_POST = do_PUT = do_DELETE = do_PATCH = _refuse


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--udp-port', type=int, default=14650)
    ap.add_argument('--cells', type=int, default=3, help='cell count if the FC does not report it')
    ap.add_argument('--journal-cmd', default='journalctl -fu drone-telemetry -n 1 -o cat --no-pager',
                    help="fallback source (old reader); 'off' to disable")
    args = ap.parse_args()
    OPTS['cells'] = args.cells
    threading.Thread(target=udp_listener, args=(args.udp_port,), daemon=True).start()
    if args.journal_cmd != 'off':
        threading.Thread(target=journal_reader, args=(shlex.split(args.journal_cmd),), daemon=True).start()
    threading.Thread(target=pi_health_loop, daemon=True).start()
    threading.Thread(target=monitor_loop, daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.daemon_threads = True
    print('drone dashboard %s on http://%s:%d  (fast feed udp 127.0.0.1:%d)' % (VERSION, args.host, args.port, args.udp_port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
