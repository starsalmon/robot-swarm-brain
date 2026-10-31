# YOLO-World detector (NUC RTX 2060 container by default, or the Windows RTX 3080 PC)

NUC: `docker compose --profile detector up -d --build detector` (see docker-compose.yml). Below is the Windows PC setup.

Open-vocabulary detection (Ultralytics YOLO-World, `yolov8x-worldv2.pt` by default) for the robots. The brain
POSTs its camera frames; the PC can also pull the Rover (192.168.3.128:81) and Beeper (192.168.3.241:81) MJPEG streams itself.

## Setup (once)
1. Install **Python 3.11** (python.org, tick "Add to PATH" and the `py` launcher). Up-to-date NVIDIA driver (CUDA 12.4+ capable).
2. Copy this folder to the PC, e.g. `C:\robot-detector`.
3. Admin PowerShell in that folder: `powershell -ExecutionPolicy Bypass -File setup.ps1`
   (creates `.venv`, installs CUDA torch cu124 + requirements, prints `CUDA True NVIDIA GeForce RTX 3080`, opens TCP 8765 on Private networks).
4. Make sure the network profile is **Private** (Settings > Network > Properties), or the firewall rule won't apply.
5. The PC is on 192.168.2.x and the robots on 192.168.3.x: the router must route between them (UniFi does by default unless isolated).

## Run
`start.bat` (first run downloads the model, ~140 MB, plus CLIP). Faster: `set DET_MODEL=yolov8l-worldv2.pt` before `start.bat`.
Env: `DET_PORT` (8765), `DET_CONF` (0.25), `DET_IMGSZ` (640), `DET_PULL` (1 = pull both cams), `DET_CAMS` (JSON name->url).

## API
- `GET /health`, `GET /vocab`, `POST /vocab {"vocab":[...]}` / `{"add":[...]}` / `{"reset":true}`
- `POST /detect?cam=rover&track=1` with JPEG body; `GET /latest?cam=beeper`
- Each detection: `cls`, `conf`, `bbox` [x1,y1,x2,y2] px, `id` (track id, -1 untracked); response has `ts`, `w`, `h`, `infer_ms`.

Check from the NUC: `curl http://192.168.2.44:8765/health`
