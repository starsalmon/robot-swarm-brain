import json, time, re, os
path = "/data/swarm/rover-telem/2026-10-02.jsonl"

def last():
    last = None
    with open(path) as f:
        for line in f:
            if "DIAG" in line:
                last = line
    return json.loads(last)

print("watch 50s for DIAG advances")
prev = last()
advances = 0
for i in range(50):
    time.sleep(1)
    o = last()
    age = time.time() - o["recv_ts"]
    if o["recv_ts"] > prev["recv_ts"]:
        advances += 1
        yaw = re.search(r"yaw=([-\d.]+)", o["line"]).group(1)
        tms = re.search(r"t=(\d+)", o["line"]).group(1)
        tof = re.search(r"tof=([-\d.,]+)", o["line"]).group(1)
        print(f"+{i+1}s NEW age={age:.1f} t={tms} yaw={yaw} tof={tof}")
        prev = o
    elif (i + 1) % 5 == 0:
        print(f"+{i+1}s wait age={age:.1f} advances={advances}")
print("TOTAL_ADVANCES", advances)
o = last()
print("FINAL_age", round(time.time() - o["recv_ts"], 1))
print("FINAL", o["recv_iso"], o["line"])
print("ping", os.system("ping -c1 -W1 192.168.3.71 >/dev/null") == 0)
