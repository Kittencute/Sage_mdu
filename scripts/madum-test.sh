#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
echo "Sending MADUM test mission..."
python3 scripts/madum_test_mission.py
