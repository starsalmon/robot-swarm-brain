#!/bin/sh
# Grow root filesystem after the hypervisor expands the VM disk.
# Run on dockerhost as root:  sh ~/robot-fleet/expand-root.sh
set -eu

DEV=/dev/sda
PART="${DEV}2"
PV="${PART}"
LV=/dev/vg0/lv_root
MNT=/

echo "== Before =="
df -hT "$MNT"
pvs "$PV" || true
lvs "$LV" || true

echo "== Rescan block device =="
echo 1 > /sys/class/block/sda/device/rescan 2>/dev/null || true
sleep 2

echo "== Grow partition 2 =="
if command -v growpart >/dev/null 2>&1; then
  growpart "$DEV" 2
elif command -v sgdisk >/dev/null 2>&1 && command -v parted >/dev/null 2>&1; then
  sgdisk -e "$DEV"
  parted -s "$DEV" resizepart 2 100%
  partprobe "$DEV" 2>/dev/null || true
else
  echo "Install partition tools: apk add parted sgdisk"
  exit 1
fi
sleep 1

echo "== Grow LVM =="
pvresize "$PV"
lvextend -l +100%FREE "$LV"

echo "== Grow ext4 =="
if ! command -v resize2fs >/dev/null 2>&1; then
  apk add --no-cache e2fsprogs-extra || apk add --no-cache e2fsprogs
fi
resize2fs "$LV"

echo "== After =="
df -hT "$MNT"
pvs "$PV"
lvs "$LV"
echo "Done."
