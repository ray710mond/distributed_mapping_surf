# Interactive shells opened with `docker compose exec surf bash` source every
# overlay needed by the sensor driver and SURF stack.
source /opt/ros/jazzy/setup.bash
source /opt/livox_ws/install/setup.bash
source /opt/px4_ws/install/local_setup.bash
# Preserve the explicitly sourced Livox and PX4 overlays when adding SURF.
source /opt/surf/local_setup.bash
# Prefer the single conventional workspace overlay once it has been built.
if [[ -f /opt/surf_ws/install/local_setup.bash ]]; then
  source /opt/surf_ws/install/local_setup.bash
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/data/ros-logs}"

PS1='surf@\h:\w\$ '
