# Robot integration

## Boundary

The portable launch starts only:

- the local Bonxai map server when `enable_mapping` is true;
- the drone sender or humanoid receiver;
- an odometry normalizer that republishes the configured robot-owned odometry
  under the existing SURF transport contract;
- the HaLow data and HaLow control `network_bridge` instances when `enable_transport` is
  true.
- the data tracker when `enable_tracker` is true.

It does not start sensor drivers, localization, PX4, robot control, RViz, or a
static transform publisher.

## Required data

### Point cloud

Configure `integration.pointcloud_topic` to a
`sensor_msgs/msg/PointCloud2`. Its timestamp must correspond to a pose that is
available through the selected pose source. Do not use an unstamped or
wall-clock-rewritten relay.

For mapping, the point cloud may be in the sensor or body frame as long as the
robot publishes the corresponding TF into `integration.map_frame`.

### Pose

The sender supports two unchanged pose modes:

- `odometry`: consume `integration.odometry_topic` directly; or
- `tf`: look up the cloud frame in `integration.map_frame`.

Use `odometry` when the existing stack publishes globally aligned odometry but
does not publish the equivalent TF. The odometry message's parent frame must
match `integration.odometry_frame`.

### TF

At each cloud timestamp, this transform must resolve:

```text
map_frame -> odometry_frame -> body_frame -> cloud header.frame_id
```

Equivalent direct transforms are acceptable. SURF does not publish these
robot-owned transforms.

The normalizer publishes `/<namespace>/transport/<role>_odometry` in
`integration.map_frame`. It does not write back to the source odometry topic,
so the robot's localization graph remains authoritative.

## Configuration

`config/robot.example.yaml` uses wildcard node selectors so the same file can
be passed to nodes under any namespace. Values under the top-level
`integration` section are read by the launch file; node parameter sections are
passed unchanged to their respective existing nodes.

Transport endpoints remain in separate `network_bridge` parameter files
because that package starts two nodes with distinct link configurations.
Create one transport file per robot from the examples under
`config/transport/`.

The launch file overrides each bridge's `subscribe_namespace` and
`publish_namespace` from `integration.robot_namespace`. Therefore the G1 may
use a namespace such as `g1` without editing transport topic names; `role`
still selects the unchanged drone-sender or humanoid-receiver protocol side.

## Preflight

Before starting SURF, verify the existing robot graph:

```bash
ros2 topic info --verbose /replace/pointcloud
ros2 topic info --verbose /replace/odometry
ros2 topic echo --once /replace/odometry
ros2 run tf2_ros tf2_echo map base_link
```

Then start one robot with transport disabled and confirm local mapping first:

```bash
ros2 launch surf_portable_bringup portable.launch.py \
  role:=drone config:=config/drone.yaml enable_transport:=false
```

Only enable inter-robot transport after both local contracts pass. The current
protocol is asymmetric: the drone sends compressed map deltas and the humanoid
receives/fuses them.

## Humble migration

Create a dedicated branch before modifying copied ROS source for Humble. Test
package manifests, ROS APIs, QoS behavior, Python versions, mapping dependency
versions, and `network_bridge` availability there. Do not mix those changes
into the Jazzy extraction commit.
