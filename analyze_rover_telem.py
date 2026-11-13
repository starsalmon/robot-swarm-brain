#!/usr/bin/env python3
"""Quick stats from rover UDP DIAG lines (e.g. ~/rover-telem/*.jsonl on the brain host)."""
from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

DIAG_RE = re.compile(
    r'sess=(\d+).*cal=(\d+).*rng=([\d.]+).*cmd=([-\d.]+),([-\d.]+).*mot=([-\d.]+),([-\d.]+)'
)


def parse_line(line: str) -> dict | None:
    m = DIAG_RE.search(line)
    if not m:
        return None
    return {
        'sess': int(m.group(1)),
        'cal': int(m.group(2)),
        'rng': float(m.group(3)),
        'cmd_lin': float(m.group(4)),
        'cmd_ang': float(m.group(5)),
        'mot_l': float(m.group(6)),
        'mot_r': float(m.group(7)),
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('jsonl', type=Path, nargs='?', default=Path('/root/rover-telem'))
    args = p.parse_args()

    paths: list[Path]
    if args.jsonl.is_dir():
        paths = sorted(args.jsonl.glob('*.jsonl'))
    else:
        paths = [args.jsonl]

    rows: list[dict] = []
    for path in paths:
        for raw in path.read_text().splitlines():
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError:
                continue
            row = parse_line(obj.get('line', ''))
            if row:
                rows.append(row)

    sess = [r for r in rows if r['sess'] == 1 and r['cal'] == 0]
    print(f'samples total={len(rows)}  session+cal_idle={len(sess)}')
    if not sess:
        return

    zero_cmd = sum(1 for r in sess if abs(r['cmd_lin']) < 0.01 and abs(r['cmd_ang']) < 0.01)
    moving = sum(1 for r in sess if abs(r['mot_l']) > 0.05 or abs(r['mot_r']) > 0.05)
    close = sum(1 for r in sess if r['rng'] < 0.5)
    print(f'zero cmd_vel: {zero_cmd}/{len(sess)} ({100*zero_cmd/len(sess):.0f}%)')
    print(f'motors moving: {moving}/{len(sess)} ({100*moving/len(sess):.0f}%)')
    print(f'sonar rng<0.5m: {close}/{len(sess)} ({100*close/len(sess):.0f}%)')

    cmd_bins = Counter()
    for r in sess:
        key = f"{r['cmd_lin']:.2f},{r['cmd_ang']:.2f}"
        cmd_bins[key] += 1
    print('top cmd_vel:')
    for k, n in cmd_bins.most_common(8):
        print(f'  {k}: {n}')


if __name__ == '__main__':
    main()
