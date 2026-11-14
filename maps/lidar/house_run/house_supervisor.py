#!/usr/bin/env python3
"""Host-side house mapping supervisor. Starts the map-based frontier planner (rover_frontier.py, a steering
hint only) and the existing rover-behaviour explore, watches health, snapshots the map, and always stops
Rover before exit. Stops when the frontier planner says the reachable house is covered."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

MAP_HOST = Path("/data/swarm/robot-swarm-brain/maps/lidar")
RUN_HOST = MAP_HOST / "house_run"
RUN_CT = "/maps/lidar/house_run"
MAP_CT = "/maps/lidar"
SLAM = "rover-lidar-slam"
BEH = "rover-behaviour"
DURATION_S = int(os.environ.get("HOUSE_DURATION_S", "1200"))  # 20 minutes (battery cutoff / coverage usually end it first)
SNAPSHOT_EVERY_S = int(os.environ.get("HOUSE_SNAPSHOT_EVERY_S", "180"))
# Never use house_status_node scan_age as a stop reason (TF/executor can stall that age).
# Lidar health comes from lidar_watch.py writing lidar_health.json off /rover/scan_sync.
LIDAR_DEAD_S = 15.0
LIDAR_DEAD_HITS_NEEDED = 4
STATUS_STALE_S = 6.0
FRONT_STALE_S = 3.0
BATTERY_MIN_V = float(os.environ.get("BATTERY_MIN_V", "6.4"))  # Cain 2026-11-14: Rover has its own LV cutoff; run with 0
STUCK_S = 90.0
STUCK_MOVE_M = 0.20
NO_GO_S = 20.0
DEADLINE_PAD_S = 60.0
COVERAGE_SNAP_MIN_S = 60.0
COVERAGE_GROW_CELLS = 400
FRONTIER_STALE_S = 10.0          # planner process dead/hung this long -> stop (it never drives, but without it Rover loops)
FRONTIER_DONE_S = 45.0           # "done" this long while running -> house covered, stop. "unreachable" means frontiers exist but no path from here; keep going so unstick can work.
REQUIRE_MAP_POSE = os.environ.get("HOUSE_ALLOW_REACTIVE", "0") != "1"

RUN_HOST.mkdir(parents=True, exist_ok=True)
LOG = RUN_HOST / "supervisor.log"
SUMMARY = RUN_HOST / "summary.json"
STOP_FLAG = RUN_HOST / "stop.flag"
HB = RUN_HOST / "supervisor.heartbeat"
MEL = timezone(timedelta(hours=11))

started = False
stopped = False
stop_reason = None
events = []
last_pose = None
last_pose_move_t = None
motion_seen = False
t_go = None
snapshots = []
peak_free = 0
peak_size = [0, 0]
lidar_dead_hits = [0]
last_free_snap = [0]


def log(msg: str) -> None:
    line = f"{datetime.now(tz=MEL).strftime('%H:%M:%S')} AEDT {msg}"
    print(line, flush=True)
    with LOG.open("a") as f:
        f.write(line + "\n")


def event(msg: str) -> None:
    events.append({"t": datetime.now(tz=MEL).isoformat(), "msg": msg})
    log(msg)


def sh(cmd, timeout=30):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)


def docker_ok(name: str) -> bool:
    r = sh(["docker", "inspect", "-f", "{{.State.Running}}", name], timeout=10)
    return r.returncode == 0 and r.stdout.strip() == "true"


def ros(cmd: str, timeout=40):
    full = f"source /opt/ros/jazzy/setup.bash && {cmd}"
    return sh(["docker", "exec", SLAM, "bash", "-lc", full], timeout=timeout)


def start_frontier():
    sh(["docker", "exec", SLAM, "bash", "-lc", "pkill -INT -f rover_frontier.py || true"], timeout=10)
    time.sleep(1.0)
    (RUN_HOST / "frontier.json").unlink(missing_ok=True)
    r = sh(["docker", "exec", "-d", SLAM, "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash && exec python3 -u /maps/lidar/house_run/rover_frontier.py >> /maps/lidar/house_run/frontier.log 2>&1"],
           timeout=15)
    log(f"frontier planner start rc={r.returncode} {(r.stdout or '')[-200:]}")
    return r.returncode == 0


def stop_frontier():
    sh(["docker", "exec", SLAM, "bash", "-lc", "pkill -INT -f rover_frontier.py || true"], timeout=10)


def read_frontier():
    p = RUN_HOST / "frontier.json"
    try:
        doc = json.loads(p.read_text())
    except Exception:
        return None, None
    return doc, time.time() - float(doc.get("wall_t") or 0)


frontier_done_since = [None]


def pub_button_true():
    return ros('python3 /maps/lidar/house_run/pub_cmd.py button', timeout=25)


def pub_estop():
    # button_event 3 always forces go=False (does not toggle)
    return ros('python3 /maps/lidar/house_run/pub_cmd.py estop', timeout=25)

LIDAR_PAUSE_WAIT_S = 90.0


def pause_for_lidar(reason: str) -> bool:
    """Stop the wheels, wait for scans to return, then tap Go. True if resumed."""
    event(f"PAUSE: {reason} — motors held until lidar returns")
    try:
        r = pub_estop()
        log(f"lidar-pause estop rc={r.returncode}")
    except Exception as e:
        log(f"lidar-pause estop failed: {e}")
    lidar_dead_hits[0] = 0
    t_wait = time.time()
    while time.time() - t_wait < LIDAR_PAUSE_WAIT_S:
        write_hb()
        if STOP_FLAG.exists():
            return False
        if scan_topic_alive(3.0):
            event("lidar is back; tapping Go")
            try:
                r = pub_button_true()
                log(f"go after lidar-pause rc={r.returncode} {(r.stdout or '')[-200:]}")
            except Exception as e:
                log(f"go after lidar-pause failed: {e}")
                return False
            t_g = time.time()
            while time.time() - t_g < 8:
                write_hb()
                doc, _st = read_live()
                if doc and (doc.get("go") or doc.get("session")):
                    event("resumed after lidar pause")
                    return True
                time.sleep(0.3)
            event("Go after lidar pause not seen in 8s; retrying once")
            try:
                pub_button_true()
            except Exception as e:
                log(f"go retry after lidar-pause failed: {e}")
            return True
        time.sleep(0.5)
    event(f"lidar did not return after {int(LIDAR_PAUSE_WAIT_S)}s pause")
    return False



def read_live(max_age=2.5):
    p = RUN_HOST / "live.json"
    if not p.exists():
        return None, "missing"
    try:
        doc = json.loads(p.read_text())
    except Exception as e:
        return None, f"badjson {e}"
    age = time.time() - float(doc.get("wall_t") or 0)
    if age > max_age:
        return doc, f"stale {age:.1f}s"
    return doc, "ok"


def read_lidar_health():
    """Freshness of a real /rover/scan_sync or /rover/scan publication, not status-node callback age."""
    p = RUN_HOST / "lidar_health.json"
    if not p.exists():
        return None, None, "missing"
    try:
        doc = json.loads(p.read_text())
    except Exception as e:
        return None, None, f"badjson {e}"
    ages = []
    topics = doc.get("topics") or {}
    for name in ("scan_sync", "scan"):
        rec = topics.get(name) or {}
        wall = rec.get("wall_t")
        if wall:
            ages.append(time.time() - float(wall))
    if not ages:
        return doc, None, "no_msgs"
    age = min(ages)
    return doc, age, "ok"


def start_lidar_watch():
    """Detached in the SLAM container so a closed stdin cannot kill it."""
    sh(["docker", "exec", SLAM, "bash", "-lc", "pkill -f lidar_watch.py || true"], timeout=10)
    time.sleep(0.2)
    r = sh(
        [
            "docker",
            "exec",
            "-d",
            SLAM,
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && python3 /maps/lidar/house_run/lidar_watch.py >> /maps/lidar/house_run/lidar_watch.log 2>&1",
        ],
        timeout=15,
    )
    log(f"lidar watch start rc={r.returncode} {(r.stdout or '')[-200:]}")
    return r.returncode == 0


def scan_topic_alive(timeout=3.0) -> bool:
    """Re-read scan_stamp health for up to timeout seconds. No new ROS node (discovery is too slow / flaky)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        _lh, age, st = read_lidar_health()
        src = None if not _lh else _lh.get("source")
        log(f"lidar recheck st={st} age={age} source={src}")
        if age is not None and age < 2.0:
            return True
        time.sleep(0.4)
    return False


def wait_live(timeout=25):
    t0 = time.time()
    while time.time() - t0 < timeout:
        doc, st = read_live(max_age=3.0)
        if doc and st == "ok":
            return doc
        time.sleep(0.3)
    return None


def save_snapshot(name: str) -> dict:
    write_hb()
    dest = MAP_CT + "/" + name
    event(f"saving snapshot {name}")
    r1 = ros(
        f"ros2 run nav2_map_server map_saver_cli -f {dest} --ros-args -p save_map_timeout:=15.0",
        timeout=40,
    )
    r2 = ros(f"python3 {MAP_CT}/render_map_png.py {dest}", timeout=40)
    info = {
        "name": name,
        "t": datetime.now(tz=MEL).isoformat(),
        "map_saver": (r1.stdout or "")[-800:],
        "render": (r2.stdout or "")[-800:],
        "ok": r1.returncode == 0 and r2.returncode == 0,
    }
    # latest copy
    src_png = MAP_HOST / f"{name}.png"
    if src_png.exists():
        latest = RUN_HOST / "latest.png"
        latest.write_bytes(src_png.read_bytes())
        host_copy = MAP_HOST / "house_latest.png"
        host_copy.write_bytes(src_png.read_bytes())
    fpng = RUN_HOST / "frontier_latest.png"
    if fpng.exists():
        try:
            (MAP_HOST / f"{name}_frontier.png").write_bytes(fpng.read_bytes())
            info["frontier_png"] = str(MAP_HOST / f"{name}_frontier.png")
        except Exception:
            pass
    snapshots.append(info)
    log(f"snapshot {name} ok={info['ok']}\n{(r2.stdout or '').strip()}")
    write_hb()
    return info


def _fsum():
    d, age = read_frontier()
    if not d:
        return None
    return (f"{d.get('state')} goal={d.get('goal')} brg={d.get('bearing_deg')} path={d.get('path_m')} "
            f"goals={d.get('n_goals')} fcells={d.get('n_frontier_cells')} visited_m2={d.get('visited_m2')} "
            f"bl={len(d.get('blacklist') or [])} age={None if age is None else round(age, 1)}")


def write_hb():
    HB.write_text(str(time.time()))


def request_stop(reason: str):
    global stopped, stop_reason
    if stop_reason is None:
        stop_reason = reason
    event(f"STOP requested: {reason}")
    # Always use e-stop event so we cannot accidentally toggle explore back on.
    try:
        r = pub_estop()
        log(f"estop pub rc={r.returncode} {(r.stdout or '')[-300:]}")
    except Exception as e:
        log(f"estop pub failed: {e}")
    # confirm
    deadline = time.time() + 12
    ok = False
    last = None
    while time.time() < deadline:
        doc, st = read_live(max_age=4.0)
        last = doc
        if doc:
            go = doc.get("go")
            ses = doc.get("session")
            v = doc.get("beh_v") or 0
            cmd = (doc.get("cmd") or {}).get("v") or 0
            if go is False and ses is False and abs(v) < 0.01 and abs(cmd) < 0.01:
                ok = True
                break
        time.sleep(0.4)
    if not ok:
        try:
            r = pub_estop()
            log(f"estop retry rc={r.returncode}")
        except Exception as e:
            log(f"estop retry failed: {e}")
        time.sleep(1.5)
        doc, st = read_live(max_age=4.0)
        last = doc
        if doc and doc.get("go") is False and doc.get("session") is False:
            ok = True
    stopped = True
    event(f"stop confirm ok={ok} go={None if not last else last.get('go')} session={None if not last else last.get('session')} v={None if not last else last.get('beh_v')}")
    return ok


def handle_signal(signum, _frame):
    request_stop(f"signal {signum}")
    raise SystemExit(1)


def pose_xy(doc):
    p = doc.get("pose") if doc else None
    if not p:
        return None
    return float(p["x"]), float(p["y"])


def evaluate(doc, t0) -> str | None:
    """Return a stop reason or None to continue."""
    global last_pose, last_pose_move_t, motion_seen, peak_free, peak_size
    if not docker_ok(BEH):
        return "rover-behaviour container not running"
    if not docker_ok(SLAM):
        return "rover-lidar-slam container not running"
    if doc is None:
        return "live telemetry missing"
    why_s = str(doc.get("why") or "")
    if "WEDGED" in why_s and "motors stopped" in why_s:   # 2026-11-13: rover_behaviour gave up after wedge escapes
        return "physically wedged: " + why_s[why_s.find("WEDGED"):][:200]
    ages = doc.get("ages") or {}
    if doc.get("allow") is not True:
        return "allow_drive became false (not touched)"
    _lh, lidar_age, lidar_st = read_lidar_health()
    if lidar_age is None or lidar_age > LIDAR_DEAD_S:
        grace = (t_go is not None and time.time() - t_go < 12)
        need = 10.0 if grace else LIDAR_DEAD_S
        if lidar_age is None or lidar_age > need:
            lidar_dead_hits[0] += 1
            if lidar_dead_hits[0] >= LIDAR_DEAD_HITS_NEEDED:
                # Watcher/file can go stale even when the topic is live. Confirm before stopping.
                if scan_topic_alive(3.0):
                    event(f"lidar watch stale ({lidar_st} age={lidar_age}) but topic is live; restarting watch, not stopping")
                    # Lidar health is written by the long-lived scan_stamp.py restamper (already subscribed).
    # Do not start lidar_watch.py: a second Python subscriber in this container can go silent and overwrite health.
                    lidar_dead_hits[0] = 0
                else:
                    return f"PAUSE:lidar dropout ({lidar_st} age={lidar_age})"
        else:
            lidar_dead_hits[0] = 0
    else:
        lidar_dead_hits[0] = 0
    if ages.get("status") is None or ages["status"] > STATUS_STALE_S:
        return f"behaviour status stale ({ages.get('status')})"
    if ages.get("front") is None or ages["front"] > FRONT_STALE_S:
        return f"front ToF stale ({ages.get('front')})"
    fdoc, fage = read_frontier()
    if fage is None or fage > FRONTIER_STALE_S:
        return f"frontier planner silent ({None if fage is None else round(fage, 1)}s)"
    fst = fdoc.get("state")
    # Only "done" means the reachable house is covered. "unreachable" means frontier
    # cells still exist but no path from the current pose (boxed in) — do not end the run.
    if fst == "done":
        frontier_done_since[0] = frontier_done_since[0] or time.time()
        if time.time() - frontier_done_since[0] > FRONTIER_DONE_S:
            return f"house covered: frontier state '{fst}' for {FRONTIER_DONE_S:.0f}s (frontier cells={fdoc.get('n_frontier_cells')})"
    else:
        frontier_done_since[0] = None
    batt = doc.get("batt")
    if batt is not None and batt < BATTERY_MIN_V:
        return f"battery low {batt:.2f} V"
    if t_go and time.time() - t_go > NO_GO_S and not doc.get("session") and not doc.get("go"):
        return "session never started (Go tap did not take)"
    v = abs(doc.get("beh_v") or 0) + abs((doc.get("cmd") or {}).get("v") or 0)
    w = abs(doc.get("beh_w") or 0)
    if v > 0.02 or w > 0.02:
        motion_seen = True
    p = pose_xy(doc)
    if p is not None:
        if last_pose is None:
            last_pose = p
            last_pose_move_t = time.time()
        else:
            dx = p[0] - last_pose[0]
            dy = p[1] - last_pose[1]
            dist = (dx * dx + dy * dy) ** 0.5
            if dist >= STUCK_MOVE_M:
                last_pose = p
                last_pose_move_t = time.time()
        # Only treat as stuck when a real pose is present and has not moved.
        if (
            motion_seen
            and last_pose_move_t is not None
            and time.time() - last_pose_move_t > STUCK_S
            and time.time() - t0 > 60
        ):
            return f"stuck: pose moved < {STUCK_MOVE_M} m for {STUCK_S:.0f}s"
    else:
        # Missing pose / broken TF is not a stuck fault. Pause the clock.
        if last_pose_move_t is not None:
            last_pose_move_t = time.time()
    info = doc.get("map") or {}
    if info:
        peak_free = max(peak_free, int(info.get("free") or 0))
        sz = info.get("size_m") or [0, 0]
        peak_size = [max(peak_size[0], sz[0]), max(peak_size[1], sz[1])]
    return None


def write_summary(extra=None):
    doc, st = read_live(max_age=8.0)
    payload = {
        "stop_reason": stop_reason,
        "started": started,
        "t_go": None if t_go is None else datetime.fromtimestamp(t_go, tz=MEL).isoformat(),
        "stopped": stopped,
        "motion_seen": motion_seen,
        "duration_s_planned": DURATION_S,
        "snapshots": snapshots,
        "peak_free": peak_free,
        "peak_size_m": peak_size,
        "events": events,
        "final_live": doc,
        "final_frontier": read_frontier()[0],
        "live_status": st,
    }
    if extra:
        payload.update(extra)
    SUMMARY.write_text(json.dumps(payload, indent=2, default=str))


def main():
    global started, t_go
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    STOP_FLAG.unlink(missing_ok=True)
    log(f"supervisor start duration={DURATION_S}s snapshot_every={SNAPSHOT_EVERY_S}s")
    write_hb()

    if not docker_ok(SLAM):
        event("ABORT: slam container down")
        write_summary()
        return 2
    if not docker_ok(BEH):
        event("ABORT: behaviour container down")
        write_summary()
        return 2

    # ensure status node + independent lidar watch
    ros("mkdir -p /maps/lidar/house_run", timeout=10)
    sh(["docker", "exec", SLAM, "bash", "-lc", "pkill -f house_status_node.py || true; pkill -f lidar_watch.py || true"], timeout=10)
    time.sleep(0.4)
    proc = subprocess.Popen(
        [
            "docker",
            "exec",
            SLAM,
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && python3 /maps/lidar/house_status_node.py",
        ]
    )
    (RUN_HOST / "status_pid").write_text(str(proc.pid))
    event(f"status node docker exec pid={proc.pid}")
    # Lidar health is written by the long-lived scan_stamp.py restamper (already subscribed).
    # Do not start lidar_watch.py: a second Python subscriber in this container can go silent and overwrite health.
    t_ready = time.time()
    doc = None
    while time.time() - t_ready < 30:
        doc, st = read_live(max_age=3.0)
        _lh, lidar_age, lidar_st = read_lidar_health()
        if doc and st == "ok":
            ages = doc.get("ages") or {}
            if (
                doc.get("allow") is not None
                and ages.get("front") is not None
                and ages.get("status") is not None
                and lidar_age is not None
                and lidar_age < 3.0
            ):
                break
        time.sleep(0.3)
    else:
        event("ABORT: no complete live telemetry (status node or lidar watch)")
        write_summary()
        return 2

    allow = doc.get("allow")
    event(
        f"preflight allow={allow} (untouched) session={doc.get('session')} go={doc.get('go')} "
        f"state={doc.get('state')} why={doc.get('why')} batt={doc.get('batt')} "
        f"tof={doc.get('tof')} scan={doc.get('scan')} scan_age={doc.get('ages',{}).get('scan')} "
        f"pose={doc.get('pose')} map={doc.get('map')}"
    )
    if allow is not True:
        event("ABORT: /fleet/allow_drive is not true; not flipping it")
        write_summary()
        return 3
    _lh, lidar_age, lidar_st = read_lidar_health()
    event(f"lidar health st={lidar_st} age={lidar_age} topics={None if not _lh else list((_lh.get('topics') or {}).keys())}")
    if lidar_age is None or lidar_age > LIDAR_DEAD_S:
        event("ABORT: /rover/scan_sync (and /rover/scan) not publishing")
        write_summary()
        return 2
    if (doc.get("ages") or {}).get("front") is None or doc["ages"]["front"] > FRONT_STALE_S:
        event("ABORT: front ToF not live")
        write_summary()
        return 2

    # map-based planner (hint only). Needs a map pose; without one the run would be the old reactive loop.
    start_frontier()
    fdoc = None
    t_f = time.time()
    while time.time() - t_f < 30:
        write_hb()
        fdoc, fage = read_frontier()
        if fdoc and fage is not None and fage < 2.0 and fdoc.get("state") in ("goal", "done", "unreachable"):
            break
        time.sleep(0.5)
    event(f"frontier planner: {json.dumps(fdoc) if fdoc else 'no status'}")
    if not fdoc or fdoc.get("state") not in ("goal", "done", "unreachable"):
        if REQUIRE_MAP_POSE:
            event("ABORT: frontier planner has no map pose/plan (TF map->base_link or /map missing). "
                  "Fix SLAM or set HOUSE_ALLOW_REACTIVE=1 for the old reactive explore.")
            stop_frontier()
            write_summary()
            return 2
        event("WARN: no frontier plan; continuing with reactive explore (HOUSE_ALLOW_REACTIVE=1)")
    elif fdoc.get("state") == "done":
        event("NOTE: planner reports no frontiers left at start; will stop after the done window if it stays that way")

    # baseline snapshot without moving
    try:
        save_snapshot("house_before")
    except Exception as e:
        log(f"baseline snapshot failed (continuing): {e}")

    event("publishing Go tap to start existing explore session")
    r = pub_button_true()
    log(f"go pub rc={r.returncode} {(r.stdout or '')[-300:]}")
    t_go = time.time()
    started = True
    # wait for session; retry once if the first pub missed rover_behaviour
    ok = False
    attempts = 1
    for _ in range(100):
        write_hb()
        doc, st = read_live()
        if doc and (doc.get("go") or doc.get("session")):
            ok = True
            event(f"session started go={doc.get('go')} session={doc.get('session')} why={doc.get('why')} v={doc.get('beh_v')} cmd={doc.get('cmd')} odom={doc.get('odom')}")
            break
        if attempts == 1 and time.time() - t_go > 6:
            lr = sh(["docker", "logs", "--since", "20s", "rover-behaviour"], timeout=10)
            if "Go tap -> START" in (lr.stdout or ""):
                event("behaviour logged START but live.json lagging; waiting")
                attempts += 1
            else:
                event("Go not seen after 6s, publishing one retry")
                r = pub_button_true()
                log(f"go retry rc={r.returncode} {(r.stdout or '')[-300:]}")
                attempts += 1
                t_go = time.time()
        time.sleep(0.3)
    if not ok:
        request_stop("Go tap did not start session")
        try:
            save_snapshot("house_final")
        except Exception as e:
            log(f"final snapshot failed: {e}")
        write_summary()
        return 4

    # Verify actual motion from cmd_vel / odom, not just a Go publish return code.
    odom0 = (doc or {}).get("odom") if ok else None
    motion_ok = False
    t_mot = time.time()
    while time.time() - t_mot < 8:
        write_hb()
        d2, _st = read_live()
        if d2:
            v = abs(d2.get("beh_v") or 0) + abs((d2.get("cmd") or {}).get("v") or 0)
            w = abs(d2.get("beh_w") or 0) + abs((d2.get("cmd") or {}).get("w") or 0)
            od = d2.get("odom") or {}
            moved = False
            if odom0 and od and "x" in od and "x" in odom0:
                dx = float(od["x"]) - float(odom0["x"])
                dy = float(od["y"]) - float(odom0["y"])
                moved = (dx * dx + dy * dy) ** 0.5 >= 0.05
            if v > 0.02 or w > 0.02 or moved:
                motion_ok = True
                event(
                    f"motion confirmed v={d2.get('beh_v')} w={d2.get('beh_w')} cmd={d2.get('cmd')} "
                    f"odom={od} pose={d2.get('pose')} why={d2.get('why')}"
                )
                break
        time.sleep(0.3)
    if not motion_ok:
        event("WARN: session is on but no cmd_vel/odom motion seen in 8s (continuing; supervisor will watch)")

    t0 = t_go
    last_snap = t0
    next_log = 0
    rc = 0
    try:
        while True:
            write_hb()
            if STOP_FLAG.exists():
                request_stop("stop.flag")
                break
            now = time.time()
            if now - t0 >= DURATION_S:
                request_stop(f"completed {DURATION_S}s first circuit")
                break
            # hard safety window
            if now - t0 >= DURATION_S + DEADLINE_PAD_S:
                request_stop("deadline pad")
                break
            doc, st = read_live(max_age=3.0)
            if st != "ok":
                # brief grace then stop
                time.sleep(0.7)
                doc2, st2 = read_live(max_age=3.0)
                if st2 != "ok":
                    request_stop(f"telemetry drop ({st2})")
                    rc = 5
                    break
                doc = doc2
            reason = evaluate(doc, t0)
            if reason:
                if reason.startswith("PAUSE:"):
                    if not pause_for_lidar(reason):
                        request_stop(reason + " (did not return)")
                        rc = 5
                        break
                    continue
                request_stop(reason)
                rc = 5
                break
            free_now = int((doc.get("map") or {}).get("free") or 0)
            grew = free_now - last_free_snap[0] >= COVERAGE_GROW_CELLS
            due = now - last_snap >= SNAPSHOT_EVERY_S
            if (grew and now - last_snap >= COVERAGE_SNAP_MIN_S) or due:
                elapsed = int(now - t0)
                name = f"house_t{elapsed:03d}s"
                try:
                    save_snapshot(name)
                    last_free_snap[0] = max(last_free_snap[0], free_now)
                except Exception as e:
                    log(f"snapshot failed: {e}")
                last_snap = now
            if now >= next_log:
                tof = doc.get("tof") or {}
                _lh, lidar_age, lidar_st = read_lidar_health()
                log(
                    f"run {int(now-t0)}s go={doc.get('go')} ses={doc.get('session')} "
                    f"v={doc.get('beh_v')} w={doc.get('beh_w')} why={doc.get('why')} "
                    f"front={tof.get('front')} left={tof.get('left')} right={tof.get('right')} "
                    f"batt={doc.get('batt')} lidar={lidar_st}:{None if lidar_age is None else round(lidar_age,3)} "
                    f"cmd={doc.get('cmd')} odom={doc.get('odom')} pose={doc.get('pose')} "
                    f"free={None if not doc.get('map') else doc['map'].get('free')} "
                    f"frontier={_fsum()}"
                )
                next_log = now + 10
            time.sleep(0.4)
    except SystemExit:
        rc = 1
    except Exception as e:
        log(f"supervisor exception: {e}")
        request_stop(f"supervisor exception: {e}")
        rc = 6
    finally:
        try:
            Path("/data/swarm/robot-swarm-brain/maps/lidar/house_run/supervisor.done").write_text(str(time.time()))
        except Exception as e:
            log(f"done flag failed: {e}")
        if not stopped:
            request_stop("finally")
        try:
            save_snapshot("house_final")
        except Exception as e:
            log(f"final snapshot failed: {e}")
        write_summary()
        try:
            stop_frontier()
        except Exception:
            pass
        try:
            sh(["docker", "exec", SLAM, "bash", "-lc", "pkill -f house_status_node.py || true; pkill -f lidar_watch.py || true"], timeout=8)
        except Exception:
            pass
        event("supervisor exit")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
