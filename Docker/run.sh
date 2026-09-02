#!/usr/bin/env sh
set -eu

IMAGE_NAME="docker3dsensors"
REPO_NAME="ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation"

# Prefer: run.sh located at repo root
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
HOST_REPO="${SCRIPT_DIR}"

# Fallback: if run.sh is launched elsewhere, try to find repo in $HOME
if [ ! -d "${HOST_REPO}" ] || [ ! -d "${HOST_REPO}/Codes" ]; then
  if [ -d "${HOME}/${REPO_NAME}/Codes" ]; then
    HOST_REPO="${HOME}/${REPO_NAME}"
  else
    echo "ERROR: Could not find repo with 'Codes' folder."
    echo "Put run.sh at the repo root (${REPO_NAME}) or clone it in \$HOME/${REPO_NAME}."
    exit 1
  fi
fi

CONT_REPO="/${REPO_NAME}"
CONT_WS="${CONT_REPO}/Codes"

USER_ID="$(id -u)"
GROUP_ID="$(id -g)"

# Map serial devices if they exist, and collect their host GIDs
DEV_ARGS=""
EXTRA_GROUPS=""
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  if [ -e "$dev" ]; then
    DEV_ARGS="$DEV_ARGS --device=$dev:$dev"
    # Get the numeric GID of the device on the HOST (avoids name mismatch inside container)
    DEV_GID="$(stat -c '%g' "$dev")"
    case "$EXTRA_GROUPS" in
      *"$DEV_GID"*) ;;  # already added
      *) EXTRA_GROUPS="$EXTRA_GROUPS --group-add $DEV_GID" ;;
    esac
  fi
done

# Map raw USB bus (libusb-style devices)
[ -d /dev/bus/usb ] && DEV_ARGS="$DEV_ARGS --device=/dev/bus/usb:/dev/bus/usb"

# X11 display forwarding (Linux host)
DISPLAY_ARGS=""
if [ -n "${DISPLAY:-}" ]; then
  xhost +local:docker 2>/dev/null || true
  DISPLAY_ARGS="-e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
fi

exec docker run --rm -it \
  --net=host \
  --privileged \
  --user "${USER_ID}:${GROUP_ID}" \
  --group-add dialout \
  $EXTRA_GROUPS \
  $DEV_ARGS \
  $DISPLAY_ARGS \
  -v "${HOST_REPO}:${CONT_REPO}:rw" \
  -w "${CONT_WS}" \
  --name docker3dsensors \
  "${IMAGE_NAME}" \
  "$@"