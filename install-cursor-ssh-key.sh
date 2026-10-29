#!/usr/bin/env bash
# One-time: run ON dockerhost while logged in as root to allow Cursor SSH from your Mac.
set -eo pipefail
PUBKEY='ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICKZI6xSUZHRpmAQZfhjwzSWmKlH2gBmMSgOAFRNtoLi cursor-dockerhost'
mkdir -p ~/.ssh
chmod 700 ~/.ssh
touch ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
if grep -qF "$PUBKEY" ~/.ssh/authorized_keys 2>/dev/null; then
  echo "Key already installed."
else
  echo "$PUBKEY" >> ~/.ssh/authorized_keys
  echo "Installed Cursor SSH key for root."
fi
