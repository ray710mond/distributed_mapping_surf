# surf_slam

`surf_slam` owns the replaceable SLAM backend used when no reference PCD is
available. It launches the vendored GLIM core and ROS 2 wrapper in CPU
pose-graph mode, converts Livox `CustomMsg` scans to `PointCloud2`, and keeps
GLIM's frames and topics role-scoped.

The root repository pins:

- `surf_ws/src/distributed_mapping_surf/dependencies/glim` to
  `https://github.com/ray710mond/glim.git`
- `surf_ws/src/distributed_mapping_surf/dependencies/glim_ros2` to the official
  ROS 2 wrapper

Initialize both with the rest of the repository:

```bash
git submodule update --init --recursive
```

For a native build, install `libgtsam-points-dev` and GLIM's other prerequisites
from the koide3 PPA as described in
`surf_ws/src/distributed_mapping_surf/dependencies/glim/docs/installation.md`,
then build the workspace with the portable baseline used by the Jetson image:

```bash
colcon build --cmake-args \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_OPENCV=OFF \
  -DBUILD_WITH_CV_BRIDGE=OFF
```

`surf_bringup hardware.launch.py` no longer starts this package automatically.
Mapless hardware runs use FAST-LIO odometry and Bonxai accumulation only. GLIM
remains available as an explicit standalone experiment, but it is not part of
the default localization or map-building path.

ScanLock is started only when startup map validation finds a known-good
reference PCD. There is no runtime promotion, critical-mass trigger, PCD
generation, or version-selection mechanism.

ScanLock remains enabled only while the robot pose is inside the original PCD's
XY bounding box **and** scan overlap is at least `0.50`. After it has anchored
successfully, five consecutive failures of either condition cause the
supervisor to retain the last strongly anchored `map -> odom` correction and
stop ScanLock. The supervisor republishes that frozen correction at every
FAST-LIO cloud timestamp, so the global TF chain remains available while all
subsequent motion comes from FAST-LIO and is allowed to drift. The handoff is
permanent for that run.

The boundary policy is configured with `scanlock_exit_overlap`,
`scanlock_exit_updates`, and `scanlock_bounds_margin`.

The live 2D map is published on `/map`. In reference-map operation, the Nav2
map server publishes its immutable grid on `/map/reference`; the
`live_occupancy_projector` preserves that trusted layer, projects current
Bonxai free/occupied voxels into 2D, and expands the output bounds as the robot
explores. During a first-run session it constructs `/map` entirely from
Bonxai. Projection defaults to `0.25 <= z <= 2.0` metres and updates once per
second; these are controlled by `live_map_min_z`, `live_map_max_z`, and
`live_map_update_period_s`.

The checked-in configuration uses CPU odometry, passthrough submapping, and
the pose-graph global mapper so it is portable to both Jetson and laptop. Both
MID-360 pipelines use the calibrated lidar-to-IMU extrinsic from
`surf_bringup/config/mid360_lio.yaml` and namespaced body frames.
