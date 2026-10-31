@echo off
cd /d %~dp0
if "%DET_MODEL%"=="" set DET_MODEL=yolov8x-worldv2.pt
.venv\Scripts\python detector_server.py
pause
