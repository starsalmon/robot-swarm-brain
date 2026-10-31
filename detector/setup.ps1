# One-time setup. Run in PowerShell from this folder:  powershell -ExecutionPolicy Bypass -File setup.ps1
$ErrorActionPreference = "Stop"
py -3.11 -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
.\.venv\Scripts\pip install -r requirements.txt
.\.venv\Scripts\python -c "import torch; print('CUDA', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '')"
# Firewall: allow the detector port from the home LAN (needs an admin PowerShell; skip if already added)
New-NetFirewallRule -DisplayName "Robot detector 8765" -Direction Inbound -Protocol TCP -LocalPort 8765 -Action Allow -Profile Private -ErrorAction SilentlyContinue
