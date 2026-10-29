import json, re, time, subprocess, os

def sh(cmd, timeout=30):
    p = subprocess.run(cmd, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    return p.returncode, p.stdout

def ros(cmd, timeout=25):
    return sh("docker exec rover-brain bash -lc " + repr("source /opt/ros/jazzy/setup.bash; " + cmd), timeout=timeout)

ping_ok = os.system("ping -c 2 -W 1 192.168.3.71 >/dev/null 2>&1") == 0
path = "/data/swarm/rover-telem/2026-10-02.jsonl"

def last_diag():
    last = None
    with open(path) as f:
        for line in f:
            if "DIAG" in line:
                last = line
    return json.loads(last)

o1 = last_diag()
time.sleep(5)
o2 = last_diag()
diag_advancing = o2["recv_ts"] > o1["recv_ts"]
age = time.time() - o2["recv_ts"]
yaw = float(re.search(r"yaw=([-\d.]+)", o2["line"]).group(1))
tms = int(re.search(r"t=(\d+)", o2["line"]).group(1))
tof_diag = re.search(r"tof=([-\d.,]+)", o2["line"]).group(1)
bat = float(re.search(r"bat=([-\d.]+)", o2["line"]).group(1))

rc, imu = ros("timeout 4 ros2 topic echo /imu/data --once")
m = re.search(r"orientation_covariance:\n((?:- .*\n){9})", imu)
cov_yaw = None
if m:
    vals = [x.strip("- ").strip() for x in m.group(1).strip().splitlines()]
    cov_yaw = vals[8] if len(vals) == 9 else None
rc, hz_out = ros("timeout 7 ros2 topic hz /imu/data")
hz_m = re.findall(r"average rate:\s*([\d.]+)", hz_out)
imu_hz = float(hz_m[-1]) if hz_m else None
imu_ok = cov_yaw is not None and "nan" not in cov_yaw.lower() and imu_hz and imu_hz > 0.5

tof = {}
for t in ["/rover/tof/front", "/rover/tof/front_left", "/rover/tof/front_right", "/rover/tof/left", "/rover/tof/right"]:
    rc, out = ros("timeout 4 ros2 topic echo %s --once" % t)
    rm = re.search(r"^range:\s*(\S+)", out, re.M)
    tof[t] = rm.group(1) if rm else ("NO_RANGE" if not out.strip() else "PARSE_FAIL")

l8_ok = False
rc, out = ros("timeout 4 ros2 topic echo /rover/tof/l8_cols --once")
if "data:" in out:
    data = out.split("data:", 1)[1]
    nums = re.findall(r"-\s+(\S+)", data)
    tof["/rover/tof/l8_cols"] = nums[:8]
    l8_ok = bool(nums) and all("nan" not in x.lower() for x in nums[:8])
else:
    tof["/rover/tof/l8_cols"] = "NO_DATA"

def is_num(v):
    try:
        float(v)
        return "nan" not in str(v).lower()
    except Exception:
        return False

tof_non_nan = {k: v for k, v in tof.items() if k != "/rover/tof/l8_cols" and is_num(v)}
tof_nan = {k: v for k, v in tof.items() if k != "/rover/tof/l8_cols" and not is_num(v)}

post = {
  "stamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
  "stamp_melbourne": time.strftime("%Y-%m-%dT%H:%M:%S AEST", time.localtime()),
  "esp_pingable": ping_ok,
  "diag_advancing": diag_advancing,
  "diag_age_s": round(age, 1),
  "diag_yaw_deg": yaw,
  "diag_t_ms": tms,
  "diag_tof": tof_diag,
  "diag_bat": bat,
  "diag_line": o2["line"],
  "imu_ok": bool(imu_ok),
  "imu_hz": imu_hz,
  "imu_orient_cov_yaw": cov_yaw,
  "tof": tof,
  "tof_numeric_ok_count": len(tof_non_nan),
  "tof_nan_topics": list(tof_nan.keys()),
  "tof_l8_ok": l8_ok,
  "up_criteria": {
    "ping": ping_ok,
    "diag_resume": bool(diag_advancing or age < 15),
    "imu_valid_orient": bool(imu_ok),
    "tof_majority_non_nan": len(tof_non_nan) >= 4 and l8_ok,
  }
}
post["UP"] = all(post["up_criteria"].values())
open("/home/cain/rover_powercycle_prep_20261002/post.json", "w").write(json.dumps(post, indent=2))
print(json.dumps(post, indent=2))
