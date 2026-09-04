#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/livox_ws/install/setup.bash
source /opt/px4_ws/install/local_setup.bash
# The ROS, Livox, and PX4 underlays are sourced explicitly above. Use SURF's
# local setup so its generated parent-prefix chain cannot replace the overlays.
source /opt/surf/local_setup.bash
# The development Compose override uses the conventional workspace install
# directory. Keep the image's release build as a fallback before the first
# development build succeeds.
if [[ -f /opt/surf_ws/install/local_setup.bash ]]; then
  source /opt/surf_ws/install/local_setup.bash
fi

if [[ -n "${LIVOX_HOST_IP:-}" || -n "${LIVOX_LIDAR_IP:-}" ]]; then
  if [[ -z "${LIVOX_HOST_IP:-}" || -z "${LIVOX_LIDAR_IP:-}" ]]; then
    echo "LIVOX_HOST_IP and LIVOX_LIDAR_IP must either both be set or both be empty" >&2
    exit 1
  fi
  python3 /usr/local/bin/render_mid360_config.py \
    /config/MID360_config.json \
    /run/surf/MID360_config.json
fi

exec "$@"
