# Distributed experiment tracker

SURF records locally on each machine and merges after a run; telemetry is not bridged over the radios under test. Normal output is:

```text
experiments/<run_id>/
├── hosts/<hostname>/telemetry.sqlite3
├── summary.csv
├── timeseries.csv
├── metadata.json
├── collection.json
├── raw_events.csv              # optional: analyze_run --raw-events
└── plots/
```

`summary.csv` is the primary result: one tidy row identifies a pipeline, stage, metric, explicit unit, measurement status, count, mean, median, min/max, standard deviation, P5/P50/P95/P99, and total. `timeseries.csv` contains one-second windows by default, plus aligned positions and distance on wireless rows. SQLite is durable recovery evidence, not a normal user-facing output.

## Measurement placement

| Host | Measurements |
|---|---|
| Drone Jetson | sender sizes/counts; filter, voxel, delta, representation and compression effects; queue/stage compute times; drone map clouds; common-frame poses; the HaLow radio |
| Humanoid laptop | reception/decode/reconstruction; accepted traffic; humanoid map clouds; common-frame poses; robot radios; local analysis and visualization |

The real pose sources are `/drone/transport/{drone,humanoid}_odometry` and `/humanoid/transport/{drone,humanoid}_odometry`. Distance is calculated only when both samples have the same `frame_id` and are within the configured maximum age (500 ms offline default). The output retains 3D/planar separation, both XYZ positions, and each sample age. Stale poses and frame mismatches are labeled, never subtracted.

Wireless sampling defaults to 2 Hz. The repository-configured devices are
`wlx0cbf7400343c`/`wlx0cbf740035d4` for the shared HaLow link.
`/sys/class/net` supplies byte/packet/error/drop counters. `iw` supplies RSSI,
expected throughput, retries, failures, beacon loss, and RX miscellaneous drops.
For Morse radios, the apparent VHT channel width and bitrate are Linux S1G
compatibility mappings; they are stored as `*_compatibility_rate_mbps`, not as
physical PHY rates. If `morse_cli` or `morsectrl` is installed, the tracker also
runs `<cli> -i <interface> stats -j` locally at 1 Hz and records its numeric
firmware/rate-control fields with a `morse_` prefix. No telemetry is sent over
the measured links. Missing hardware fields remain unavailable and never crash
collection.

### Persistent Morse HaLow telemetry access

The tracker prefers snapshots of the Morse `mmrc_table_csv` and `page_stats`
debugfs endpoints. Because debugfs does not support POSIX ACLs on these hosts, a
root systemd service reads only those endpoints at 1 Hz and atomically publishes
read-only snapshots under `/run/surf-halow-telemetry/<interface>`. Install it
once on each host; it starts on every boot and tolerates driver reloads:

```bash
# Laptop
sudo ./deploy/halow-telemetry/install-surf-halow-telemetry.sh \
  raymond wlx0cbf740035d4

# Jetson
sudo ./deploy/halow-telemetry/install-surf-halow-telemetry.sh \
  <jetson-user> <jetson-halow-interface>
```

The Jetson Compose deployment bind-mounts only the exported `/run` directory
into the container read-only. Recreate the container after updating the Compose
file so its tracker can see that mount.

Verify as the normal tracker user:

```bash
head -2 "/run/surf-halow-telemetry/<interface>/mmrc_table_csv"
cat "/run/surf-halow-telemetry/<interface>/page_stats"
systemctl status surf-halow-telemetry-exporter.service --no-pager
```

For a full host deployment, the laptop and Jetson network installers also
install this exporter, a persistent power-save-off policy applied at boot and
after NetworkManager reconnects, and the `surf-preflight` health check. The
Jetson installer persists its laptop HaLow Chrony source and continuously
refreshes the clock proof mounted into the container. The laptop is configured
as an offline Chrony authority for the dedicated HaLow subnet.

## Run, stop, collect, analyze

Choose one ID and use it verbatim on the drone Jetson and humanoid laptop:

```bash
RUN_ID="range-$(date -u +%Y%m%dT%H%M%SZ)"
```

```bash
# Drone Jetson
ros2 launch surf_bringup hardware.launch.py \
  role:=drone map_name:=amberlab \
  tracker_run_id:="$RUN_ID" tracker_experiment_name:=wireless_range

# Humanoid laptop
ros2 launch surf_bringup laptop_humanoid_rviz.launch.py \
  map_name:=amberlab \
  tracker_run_id:="$RUN_ID" tracker_experiment_name:=wireless_range
```

For defensible one-way latency, both hosts must record verified clock evidence.
The laptop tracker queries local Chrony automatically at startup. On the
Jetson, `surf-jetson-test-mode` writes a fresh verification snapshot under
`/run/surf-clock-sync`; Compose mounts that snapshot into the tracker container.
Run test mode no more than five minutes before launching the experiment.
Explicit `tracker_clock_sync_method`, `tracker_clock_offset_ms`, and
`tracker_clock_uncertainty_ms` arguments override detection. If either host is
unverified, cross-host latency rows are omitted; same-process monotonic compute
durations remain valid. Each tracker writes a `clock/synchronization` event so
offset, uncertainty, and verification state appear in the normal data outputs.
For every included cross-host latency series, analysis also reports
`cross_host_clock_uncertainty_ms`, conservatively calculated as the sum of both
hosts' startup uncertainties.

Stop each launch with one `Ctrl+C` (or SIGINT/SIGTERM). The tracker stops accepting events, commits SQLite/WAL state, writes host metadata, marks the result completed/interrupted, and leaves source data intact. Docker Compose provides a 30-second shutdown grace period.

From the laptop, supply the SSH locations that are actually reachable (none are hardcoded):

```bash
ros2 run surf_data_tracker collect_run "$RUN_ID" \
  --destination ~/SURF_2026/experiments \
  --source drone=<user>@<drone-host>:<SURF_TELEMETRY_DIRECTORY> \
  --source humanoid=<user>@<humanoid-host>:<SURF_TELEMETRY_DIRECTORY>

ros2 run surf_data_tracker analyze_run ~/SURF_2026/experiments/$RUN_ID
```

On the configured SURF laptop, the installed convenience command performs both
steps and uses the repository's standard laptop and Jetson experiment roots:

```bash
surf-collate-run "$RUN_ID"
```

`SURF_PROJECT_ROOT`, `SURF_LOCAL_EXPERIMENTS`,
`SURF_EXPERIMENT_DESTINATION`, and `SURF_DRONE_EXPERIMENTS` can override its
defaults for a different checkout or SSH endpoint.

Starting the tracker with an existing run ID replaces that host's previous
database and metadata; do not run two trackers concurrently with the same ID on
one host. Collection uses `rsync --partial`, so a replacement database also
replaces its previously collated copy while unrelated host directories remain.
Failures are listed in `collection.json`; originals are not deleted and the same
command is the retry procedure. Analysis recursively finds available hosts,
deduplicates stable event IDs, tolerates missing hosts/metrics and interrupted
runs, and regenerates compact results.

Useful analysis options:

```bash
ros2 run surf_data_tracker analyze_run ~/SURF_2026/experiments/$RUN_ID \
  --window-seconds 1 --pose-max-age-ms 500 --raw-events
ros2 run surf_data_tracker analyze_run ~/SURF_2026/experiments/$RUN_ID --no-plots
```

Plots consume time windows, not raw events. Distance views are direct scatter observations; no physical or mathematical model is fitted.

## Definitions and limitations

| Layer | Exact meaning |
|---|---|
| conceptual point data | `width × height`, field layout, `point_step`, and data-buffer length; no copy |
| raw ROS size | CDR serialization of sender `PointCloud2` |
| sparse representation | deterministic SVD2 bytes before optional zstd |
| compressed payload | `CompressedVoxelDelta.payload` length |
| ROS transport message | CDR serialization of `CompressedVoxelDelta`, not on-air bytes |
| packets per update | sender count of bounded CDR transport chunks for one logical version |
| chunk loss | missing chunk indexes among receiver-observed chunks for a logical version |
| update loss | logical version gaps; completely unobserved versions, distinct from chunk loss |
| application demand/goodput | sender/accepted-receiver ROS message bytes per window |
| interface throughput | Linux counter deltas, including other traffic and protocol overhead |
| HaLow PHY state | Morse-native JSON statistics; generic `iw` rate is compatibility-only |
| compute time | same-process `steady_clock` duration |
| cross-host latency | emitted only with declared verified clock synchronization and uncertainty |

Incremental byte reduction compares adjacent representations; cumulative byte reduction compares to raw ROS serialization. Point filtering and voxelization are explicitly element-count attribution because points, voxels, and bytes are not interchangeable. Counterfactual rates are labeled calculated; interface rates are measured.

`network_bridge` 3.0.0 does not expose its private second compression/framing size, so physical application wire bytes are unavailable. Interface counters cannot attribute TCP retransmissions to one process. Bonxai exposes no integration timing hook here, so receiver integration compute cost is unavailable. Driver-specific wireless fields remain absent when the driver does not report them.

Observed interface throughput bounds describe traffic seen during this run, not unused link capacity. A defensible saturation ceiling/lower operating envelope still requires a separate controlled offered-load sweep (for example iperf) at the same distances; running that transfer during the architecture experiment would perturb the system being measured.

## Configuration and extension

Important ROS parameters are `run_id`, `experiment_name`, `notes`, `output_root`, `record_raw_events`, `network_stats_enabled`, `network_stats_rate_hz`, `morse_stats_rate_hz`, `morse_cli_path`, `network_interfaces`, `pose_topics`, `cloud_topics`, and `flush_interval_s`. Interface entries use `device=logical_link`; poses use `system=topic`; cloud observations use `pipeline=stage=topic`. `morse_cli_path` may be left empty for automatic discovery of `morse_cli` or `morsectrl`.

Add a cloud stream or wireless interface through those lists. For a new in-process stage, publish `PipelineMetrics` using monotonic timing and already-known buffer/count sizes; do not copy a cloud for instrumentation.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths surf_ws/src --symlink-install
source install/setup.bash
PYTHONPATH=src/distributed_mapping_surf/surf_data_tracker \
  python3 -m unittest discover \
    -s src/distributed_mapping_surf/surf_data_tracker/test -v
```

Real-time HaLow updates additionally use a minimal best-effort return ACK after
the receiver reconstructs all chunks. The sender records clock-independent full
update completion RTT, final-chunk RTT, ACK timeouts, and `final-chunk RTT / 2`
as an explicitly symmetric-path one-way estimate.
ACKs resolve matching packet debt; timeout leaves information pending for BACKLOG reselection.

Controller-step logging, runtime matrix revisions, capacity calibration and the
`identify_allocation` utility are documented in the [allocation guide](../docs/information_allocation.md).
