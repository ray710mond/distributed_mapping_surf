# SURF portable mapping and communications stack

This repository contains the SURF mapping and communications ROS packages.
Clone it under a ROS workspace's `src/` directory.

The extracted stack owns collaborative mapping, compressed map transport, and
telemetry. It does not start or configure a robot's lidar, localization,
locomotion, flight control, host networking, Docker runtime, or general
perception stack.

Map communication uses a continuous DELTA/BACKLOG LQR allocator over one HaLow
radio. See [architecture and experiment guide](docs/information_allocation.md)
for lifecycle, temporal correctness, tuning, capacity calibration and validation.

## Current compatibility

The copied source is the existing ROS 2 Jazzy source, without Humble-specific
changes. Treat the contents as the starting point for a future Humble branch.
No Humble compatibility is claimed yet.

## Robot contract

Each robot must provide:

- `sensor_msgs/msg/PointCloud2` on the configured `pointcloud_topic`;
- `nav_msgs/msg/Odometry` on the configured `odometry_topic`, unless the
  sender is configured to use TF directly;
- a timestamped TF tree connecting `map_frame`, `odometry_frame`,
  `body_frame`, and the point cloud's `header.frame_id`;
- synchronized wall clocks on both robots.

SURF publishes its transport, map delta, map, and diagnostic topics below the
configured robot namespace. It never remaps or replaces the source robot's
localization topics.

See [config/robot.example.yaml](config/robot.example.yaml) for the complete
integration surface and [docs/INTEGRATION.md](docs/INTEGRATION.md) for wiring
and validation instructions.

## Layout

```text
surf_*/                 First-party ROS packages
config/                 Per-robot contract examples
docs/                   Integration and extraction notes
tools/                  Configuration validation utilities
dependencies/           Required pinned Git submodules
```

Bonxai, Caltech Mapping, GLIM, and GLIM ROS are pinned Git submodules under
`dependencies/`. Their provenance and revisions are recorded in
[SOURCE_MANIFEST.md](SOURCE_MANIFEST.md).

## Build on the existing Jazzy development system

```bash
mkdir -p ~/surf_ws/src
cd ~/surf_ws
git clone --recurse-submodules \
  <distributed_mapping_surf-url> src/distributed_mapping_surf
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src \
  --skip-keys "livox_ros_driver2 fast_lio scan_lock" -r -y
colcon build --symlink-install
source install/setup.bash
```

`dependencies/network_bridge` vendors version 3.0.0 with bounded FIFO forwarding
for chunk topics. Build and source it on both hosts; the stock binary package
uses latest-value forwarding and can overwrite chunks between send ticks.
See [transport fixes](docs/transport_and_state_fixes.md) for limits and verification. `surf_bringup` and `surf_slam` are included to
preserve the complete current source set, but the portable launch does not
invoke their Livox/LIO/GLIM hardware pipeline.

Device-specific launchers, Dockerfiles, radio setup, and clock provisioning
belong in the parent system repository that consumes these packages.

## Launch against an existing robot stack

Copy the example, edit every topic/frame/address for that robot, then launch:

```bash
cp config/robot.example.yaml config/drone.yaml
ros2 launch surf_portable_bringup portable.launch.py \
  role:=drone config:=config/drone.yaml
```

On the other robot:

```bash
cp config/robot.example.yaml config/humanoid.yaml
ros2 launch surf_portable_bringup portable.launch.py \
  role:=humanoid config:=config/humanoid.yaml
```

The role names retain current behavior: `drone` produces compressed voxel
deltas, while `humanoid` receives and fuses them. This is deliberate so the
extraction does not change the current protocol.

Run `tools/validate-config` before deployment; it catches placeholders and
basic cross-field errors without requiring ROS.
