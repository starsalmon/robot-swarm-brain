#!/usr/bin/env python3
"""Analyze IMU + cmd_vel CSV from record-explore-imu.sh."""
from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass


@dataclass
class Sample:
    t: float
    gz: float
    ax: float
    ay: float
    az: float


@dataclass
class Cmd:
    t: float
    lin: float
    ang: float


def load(path: str) -> tuple[list[Sample], list[Cmd]]:
    imu: list[Sample] = []
    cmds: list[Cmd] = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            kind = row['kind']
            t = float(row['t'])
            if kind == 'imu':
                imu.append(
                    Sample(
                        t=t,
                        gz=float(row['gz']),
                        ax=float(row['ax']),
                        ay=float(row['ay']),
                        az=float(row['az']),
                    )
                )
            elif kind == 'cmd':
                if row['lin_x'] == '' or row['ang_z'] == '':
                    continue
                cmds.append(Cmd(t=t, lin=float(row['lin_x']), ang=float(row['ang_z'])))
    return imu, cmds


def nearest_cmd(cmds: list[Cmd], t: float) -> Cmd | None:
    if not cmds:
        return None
    best = min(cmds, key=lambda c: abs(c.t - t))
    if abs(best.t - t) > 0.15:
        return None
    return best


def integrate_yaw(imu: list[Sample], gyro_sign: float = -1.0) -> tuple[list[float], list[float]]:
    if len(imu) < 2:
        return [], []
    ts = [imu[0].t]
    yaw = [0.0]
    for a, b in zip(imu, imu[1:]):
        dt = b.t - a.t
        if dt <= 0 or dt > 0.1:
            continue
        yaw.append(yaw[-1] + math.degrees(a.gz * gyro_sign) * dt)
        ts.append(b.t)
    return ts, yaw


def jerk_stats(imu: list[Sample]) -> tuple[float, float]:
    if len(imu) < 3:
        return 0.0, 0.0
    jerks: list[float] = []
    for a, b, c in zip(imu, imu[1:], imu[2:]):
        dt1 = b.t - a.t
        dt2 = c.t - b.t
        if dt1 <= 0 or dt2 <= 0:
            continue
        j = abs((c.gz - b.gz) / dt2 - (b.gz - a.gz) / dt1) / (dt1 + dt2) * 2.0
        jerks.append(j)
    if not jerks:
        return 0.0, 0.0
    jerks.sort()
    return jerks[len(jerks) // 2], max(jerks)


def cmd_step_stats(cmds: list[Cmd]) -> tuple[float, float, int]:
    if len(cmds) < 2:
        return 0.0, 0.0, 0
    d_lin: list[float] = []
    d_ang: list[float] = []
    big = 0
    for a, b in zip(cmds, cmds[1:]):
        dt = b.t - a.t
        if dt <= 0 or dt > 0.2:
            continue
        dl = abs(b.lin - a.lin)
        da = abs(b.ang - a.ang)
        d_lin.append(dl / dt)
        d_ang.append(da / dt)
        if dl > 0.12 or da > 0.15:
            big += 1
    if not d_ang:
        return 0.0, 0.0, big
    d_ang.sort()
    d_lin.sort()
    return d_ang[len(d_ang) // 2], max(d_ang), big


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('--gyro-sign', type=float, default=-1.0)
    args = p.parse_args()

    imu, cmds = load(args.csv)
    if not imu:
        print('No IMU samples', file=sys.stderr)
        sys.exit(1)

    t0, t1 = imu[0].t, imu[-1].t
    duration = t1 - t0
    _, yaw = integrate_yaw(imu, args.gyro_sign)
    total_spin_deg = abs(yaw[-1] - yaw[0]) if yaw else 0.0
    full_rotations = total_spin_deg / 360.0

    gz_vals = [s.gz * args.gyro_sign for s in imu]
    gz_abs = [abs(g) for g in gz_vals]
    gz_abs.sort()
    median_gz = gz_abs[len(gz_abs) // 2]
    p95_gz = gz_abs[int(len(gz_abs) * 0.95)]

    spin_samples = 0
    for s in imu:
        c = nearest_cmd(cmds, s.t)
        if c and abs(c.lin) < 0.08 and abs(s.gz * args.gyro_sign) > 0.25:
            spin_samples += 1

    med_jerk, max_jerk = jerk_stats(imu)
    med_dang, max_dang, big_steps = cmd_step_stats(cmds)

    # IMU-only: estimate commanded-vs-actual when cmd sparse
    gz_spikes = sum(1 for g in gz_abs if g > 0.35)
    gz_quiet = sum(1 for g in gz_abs if g < 0.05)

    lin_vals = [c.lin for c in cmds]
    ang_vals = [c.ang for c in cmds]
    cmd_spin = sum(1 for c in cmds if abs(c.lin) < 0.05 and abs(c.ang) > 0.4)

    print(f'=== Drive log analysis: {args.csv} ===')
    print(f'Duration:        {duration:.1f}s')
    print(f'IMU samples:     {len(imu)}  (~{len(imu)/max(duration,1):.0f} Hz)')
    print(f'cmd_vel msgs:    {len(cmds)}  (~{len(cmds)/max(duration,1):.1f} Hz)')
    print()
    print('--- Rotation (IMU integrated yaw) ---')
    print(f'Net heading change: {total_spin_deg:.0f}°  ({full_rotations:.2f} full turns)')
    print(f'|gz| median / p95:  {median_gz:.3f} / {p95_gz:.3f} rad/s')
    print(f'On-spot spin IMU:  {spin_samples} samples (|lin|<0.08 & |gz|>0.25)')
    print(f'High |gz| spikes:   {gz_spikes} samples (>0.35 rad/s)')
    print(f'Near-still |gz|:   {gz_quiet} samples (<0.05 rad/s)')
    print()
    print('--- Jerk / smoothness ---')
    print(f'IMU gz jerk med/max: {med_jerk:.2f} / {max_jerk:.2f} (rad/s³)')
    print(f'cmd Δang rate med/max: {med_dang:.2f} / {max_dang:.2f} per s')
    print(f'cmd big steps:       {big_steps} (Δlin>0.12 or Δang>0.15)')
    print()
    print('--- Commands sent ---')
    if lin_vals:
        print(f'lin  min/mean/max: {min(lin_vals):+.2f} / {sum(lin_vals)/len(lin_vals):+.2f} / {max(lin_vals):+.2f}')
    if ang_vals:
        print(f'ang  min/mean/max: {min(ang_vals):+.3f} / {sum(ang_vals)/len(ang_vals):+.3f} / {max(ang_vals):+.3f}')
    print(f'cmd pivot commands:  {cmd_spin} (|lin|<0.05 & |ang|>0.4)')
    if cmds and imu:
        # Compare commanded angular vs gyro when driving forward
        pairs: list[tuple[float, float, float]] = []
        for s in imu:
            c = nearest_cmd(cmds, s.t)
            if c and abs(c.lin) > 0.08:
                pairs.append((c.ang, s.gz * args.gyro_sign, c.lin))
        if pairs:
            errs = [abs(p[0] - p[1]) for p in pairs[:500]]
            errs.sort()
            print()
            print('--- cmd vs IMU (forward samples) ---')
            print(f'pairs: {len(pairs)}  |cmd_ang - gz| med: {errs[len(errs)//2]:.3f} rad/s')
            print(f'cmd_ang mean: {sum(p[0] for p in pairs)/len(pairs):+.3f}  '
                  f'gz mean: {sum(p[1] for p in pairs)/len(pairs):+.3f}')
    print()
    print('--- Last 12 cmd_vel samples ---')
    for c in cmds[-12:]:
        print(f'  t={c.t-t0:6.1f}s  lin={c.lin:+.2f}  ang={c.ang:+.3f}')


if __name__ == '__main__':
    main()
