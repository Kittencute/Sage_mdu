#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SAGE_INSTANCE="${SAGE_INSTANCE:-1}"
SERVICE_USER="$(id -un)"
UNIT_DIR=/etc/systemd/system
FLEET_PYTHONPATH="${SAGE_DIR}/src/fleet_config:${SAGE_DIR}/src/fleet_unit"

TARGET="$(cat "${SAGE_DIR}/.sage/current")" \
  || { echo "install failed: operation: read the selected target from ${SAGE_DIR}/.sage/current; cause: reported above" >&2; exit 1; }
id -nG "${SERVICE_USER}" | grep -qw docker \
  || { echo "install failed: operation: check that ${SERVICE_USER} can run docker; cause: ${SERVICE_USER} is not in group docker" >&2; exit 1; }
CAN_INTERFACES="$(python3 -c '
import sys, yaml
values = yaml.safe_load(open(sys.argv[1]))[sys.argv[2]]
names = [value for key, value in values.items() if key.endswith("_can_interface")]
if not names:
    sys.exit(f"entry {sys.argv[2]} has no *_can_interface key")
print(" ".join(names))
' "${SAGE_DIR}/site.yaml" "${TARGET}")" \
  || { echo "install failed: operation: read the CAN interfaces of ${TARGET} from ${SAGE_DIR}/site.yaml; cause: reported above" >&2; exit 1; }
PTP_INTERFACE="$(python3 -c '
import sys, yaml
print(yaml.safe_load(open(sys.argv[1]))[sys.argv[2]]["ptp_interface"])
' "${SAGE_DIR}/site.yaml" "${TARGET}")" \
  || { echo "install failed: operation: read ptp_interface of ${TARGET} from ${SAGE_DIR}/site.yaml; cause: reported above" >&2; exit 1; }
RECORDING_ENABLED="$(PYTHONPATH="${FLEET_PYTHONPATH}" python3 -c '
import sys
from fleet_config.targets import TARGETS
print(int(TARGETS[sys.argv[1]].features.recording_enabled))
' "${TARGET}")" \
  || { echo "install failed: operation: read whether target ${TARGET} records; cause: reported above" >&2; exit 1; }

sudo mkdir -p /etc/sage
printf "SAGE_DIR=%s\nSAGE_INSTANCE=%s\nTARGET=%s\n" "${SAGE_DIR}" "${SAGE_INSTANCE}" "${TARGET}" \
  | sudo tee /etc/sage/env >/dev/null

sudo apt-get install -y chrony linuxptp
sudo cp "${SAGE_DIR}/docker/timesync/chrony.conf" /etc/chrony/chrony.conf
sudo cp "${SAGE_DIR}/docker/timesync/ptp4l.conf" /etc/linuxptp/ptp4l.conf
sudo systemctl enable chrony.service chrony-wait.service "ptp4l@${PTP_INTERFACE}.service"
sudo systemctl restart chrony.service "ptp4l@${PTP_INTERFACE}.service"

if [ -e "${UNIT_DIR}/can.service" ]; then
  sudo systemctl disable --now can.service
  sudo rm "${UNIT_DIR}/can.service"
fi
sudo cp "${SAGE_DIR}/docker/systemd/can@.service" "${UNIT_DIR}/"
sed "s/^User=<user>$/User=${SERVICE_USER}/" "${SAGE_DIR}/docker/systemd/sage-robot.service" \
  | sudo tee "${UNIT_DIR}/sage-robot.service" >/dev/null
if [ "${RECORDING_ENABLED}" = 1 ]; then
  sudo cp "${SAGE_DIR}/docker/systemd/sage-recording-cleanup.service" "${UNIT_DIR}/"
  sudo cp "${SAGE_DIR}/docker/systemd/sage-recording-cleanup.timer" "${UNIT_DIR}/"
elif [ -e "${UNIT_DIR}/sage-recording-cleanup.timer" ]; then
  sudo systemctl disable --now sage-recording-cleanup.timer
  sudo rm "${UNIT_DIR}/sage-recording-cleanup.timer" "${UNIT_DIR}/sage-recording-cleanup.service"
fi
sudo systemctl daemon-reload
for interface in ${CAN_INTERFACES}; do
  sudo systemctl enable "can@${interface}.service"
done
sudo systemctl enable sage-robot.service
if [ "${RECORDING_ENABLED}" = 1 ]; then
  sudo systemctl enable sage-recording-cleanup.timer
fi
echo "Services installed. Reboot or 'sudo systemctl start sage-robot' to launch."
