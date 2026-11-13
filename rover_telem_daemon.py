#!/usr/bin/env python3
"""Always-on UDP listener for ESP standalone rover telemetry (port 4243).

Writes JSONL under ROVER_TELEM_DIR (default ~/rover-telem), one file per UTC day.
Each line: {"recv_ts": epoch, "recv_iso": "...", "src": "ip:port", "line": "..."}

Run on the NUC or another LAN host that can reach the rover ESP UDP diag port.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

DIR = Path(__file__).resolve().parent
PORT = int(os.environ.get('ROVER_TELEM_PORT', '4243'))
OUT_DIR = Path(os.path.expanduser(os.environ.get('ROVER_TELEM_DIR', '~/rover-telem')))
FLUSH_EVERY = int(os.environ.get('ROVER_TELEM_FLUSH_LINES', '20'))
STATS_EVERY_S = float(os.environ.get('ROVER_TELEM_STATS_S', '60'))

_running = True


def _stop(*_args: object) -> None:
    global _running
    _running = False


def _day_path(ts: float) -> Path:
    day = datetime.fromtimestamp(ts, tz=timezone.utc).strftime('%Y-%m-%d')
    return OUT_DIR / f'{day}.jsonl'


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('', PORT))
    except OSError as exc:
        print(f'bind :{PORT} failed: {exc}', file=sys.stderr)
        return 1

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    print(f'rover-telem daemon listening UDP :{PORT} → {OUT_DIR}', flush=True)

    lines_since_flush = 0
    total = 0
    last_stats = time.monotonic()
    cur_file: Path | None = None
    fh = None

    while _running:
        sock.settimeout(1.0)
        try:
            data, addr = sock.recvfrom(4096)
        except TimeoutError:
            if time.monotonic() - last_stats >= STATS_EVERY_S:
                print(f'  … {total} lines recorded', flush=True)
                last_stats = time.monotonic()
            continue

        line = data.decode('utf-8', errors='replace').strip()
        if not line:
            continue

        now = time.time()
        rec = {
            'recv_ts': now,
            'recv_iso': datetime.fromtimestamp(now, tz=timezone.utc).isoformat(),
            'src': f'{addr[0]}:{addr[1]}',
            'line': line,
        }
        path = _day_path(now)
        if path != cur_file:
            if fh:
                fh.flush()
                fh.close()
            cur_file = path
            fh = path.open('a', encoding='utf-8')
            print(f'  → {path.name}', flush=True)

        assert fh is not None
        fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
        total += 1
        lines_since_flush += 1
        if lines_since_flush >= FLUSH_EVERY:
            fh.flush()
            lines_since_flush = 0

        if time.monotonic() - last_stats >= STATS_EVERY_S:
            print(f'  … {total} lines recorded', flush=True)
            last_stats = time.monotonic()

    if fh:
        fh.flush()
        fh.close()
    sock.close()
    print('rover-telem daemon stopped', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
