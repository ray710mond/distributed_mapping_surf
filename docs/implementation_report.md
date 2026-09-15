# Information allocator implementation report

This records the initial migration. Subsequent scheduling, telemetry and validation
changes are documented in [the indoor-test fixes](indoor_test_fixes.md).

Completed implementation and local validation on 2026-09-11. Changes are uncommitted and have not been deployed to either robot. See [the architecture and tuning guide](information_allocation.md) for the full protocol, formulas and operational instructions.

## Implemented behavior

- Removed the discrete AdaptiveModeController, its configuration/tests/report, periodic whole-map radio snapshots, and the dedicated 5 GHz map path. Both map classes now share the HaLow data bridge and one serialized-byte budget. Reliable control also uses HaLow. Unrelated Wi-Fi management remains available.
- DELTA is a newly observed occupancy-state change, including first observations outside the shared prior, clearing, dynamic expiry, and restoration of a previously cleared prior voxel. Reobserving unchanged state does not generate duplicate debt.
- BACKLOG is still-relevant information deferred by allocation, unsuccessful after an ACK timeout, or made outstanding by loss of receiver delivery knowledge. Recovery may requeue all retained entries, but delivery remains incremental and budgeted.
- One entry per coordinate starts as DELTA, remains debt in flight, moves to BACKLOG on deferral/timeout, and resolves only on its exact packet ACK. A newer changed observation supersedes the old entry across both classes. Late ACKs cannot resolve that replacement. Raw counts and weighted debt are separate.
- For class i, `p_i = base_i + age_i*clamp(pending_age/age_seconds_i,0,1) + proximity_i/(1+distance/distance_metres_i) + dynamic_i*[dynamic occupied] + destructive_i*[free or deleted]`. Proximity is omitted without a fresh common-frame peer pose. Highest score is selected first; ties use observation age then coordinate. Debt is `e_i = sum(p_i)` over pending entries, including in-flight entries.
- Sender supersession and receiver per-coordinate timestamps independently prevent older/equal observations from replacing current state. Receiver state is shared across traffic classes; tombstones retain timestamps. Bonxai and the visualizer also enforce temporal ordering. Spatially distinct observations may arrive in any priority order. Packet IDs identify attempts, not global voxel freshness. Retired epochs cannot reactivate during a process lifetime; a new epoch intentionally resets a source layer.
- State is `[delta_debt, backlog_debt]`; input is `[delta_rate, backlog_rate]` in serialized bytes/s. The model is `x_next=A*x+B*u+d`, with cost `sum(x'Qx+u'Ru)`. Eigen iterative discrete DARE computes cached K and `u_requested=-K*x`. Negative development B models debt reduction. Matrix checks cover finite/intermediate values, PSD Q, PD R, convergence, finite gain, and closed-loop spectral radius.
- A/B/Q/R are four-element row-major ROS double arrays. Atomic parameter updates validate the complete proposed configuration before swapping model/gain/revision. Invalid runtime updates retain the last valid controller; unchanged models do not repeatedly solve DARE. Malformed startup dimensions fail configuration; other invalid startup matrix values retain the development controller.
- A separate projection clamps invalid/negative requests and proportionally scales positive rates into the shared capacity. The fixed wall timer is independent of scans. Per-class and shared credits realize rates, with at most one packet of saved credit. Bounded trial encoding selects a priority prefix whose actual CDR size fits both credits and the 1200-byte transport limit.
- Every packet is independently encoded and ACKed. Timeout retries reselect current ledger data under a fresh attempt ID. Startup broadcast recovery supports arbitrary configured sender names and works without new scans. Receiver retention is replayed over local DDS for downstream restart. Map publishers use volatile durability to prevent unbudgeted DDS history replay through the bridge.

## Capacity inspection and experiment support

The installed Morse MM8108 2.0.0 driver exposes MMRC rate-control MCS/BW/GI, average/max throughput statistics, success/attempt/MPDU counters, page statistics, and mesh-only PHY/RSSI reporting. Existing exporter snapshots and Linux/iw counters are reused where present; snapshots older than three seconds are rejected. Generic nl80211 compatibility bitrates are not treated as application capacity. The live interface query returned “No such device”, so this work establishes no live radio capacity measurement.

The external capacity abstraction prefers a supplied direct application-capacity measurement, then configurable MMRC conversion, then an explicitly labeled development fallback. The shipped ROS provider supplies the latter two paths; direct application measurement is an extension input to the pure abstraction. Achieved throughput never substitutes for capacity. Factor zero disables uncalibrated conversion. Raw telemetry, conversion factor/method, derived capacity, offered CDR load, and achieved interface throughput remain distinct. Once received, stale/invalid capacity telemetry makes sender map capacity zero.

Tracker additions record state/counts, cumulative debt events and disturbance, matrices/gain/revisions, requested/allocated rates, capacity/projection, selected priorities, codecs and byte counts, computation timing, ACK behavior, observation ages/known acknowledged-state lag, supersession and stale rejection. Consecutive rows provide next-state and queue-growth measurements. Applied temporal regressions are required to remain zero; analysis fails on a nonzero value. Combined-class packet sequence analysis avoids false loss from normal DELTA/BACKLOG interleaving.

`identify_allocation` exports one step CSV, identification JSON and two figures. Scaled least squares fits `[x u] -> x_next-d`, excluding epoch/model boundaries, missing steps and excessive timing jitter. It reports rank, conditioning, sample count, RMSE and validity warnings. Figures and existing tracker analysis support debt, allocation, capacity, offered/achieved load, queue, priority, retry/loss and Q/R tradeoff evaluation. There is no automatic online adaptation or fitted distance law.

## Parameters and initial tuning

- `allocation.dt=0.1`; `allocation.a=I`, `.b=-0.001 I`, `.q=I`, `.r=0.0001 I`. All matrices need experimental identification/tuning.
- `scheduling.delta_hz=10`, `scheduling.backlog_hz=10`.
- `delivery.defer_seconds=0.2`, `delivery.ack_timeout_seconds=2.0`.
- `priority.{delta,backlog}.{base,age,proximity,dynamic,destructive,age_seconds,distance_metres}`: weights one except BACKLOG age two; normalizations 10 seconds/10 metres.
- `capacity.telemetry_timeout_seconds=3`. The capacity provider exposes `interface` and `experimental_mmrc_factor=0.2`; launch exposes corresponding capacity arguments. Missing/stale MMRC telemetry grants zero map credit.
- Matrix parameters support runtime updates. Other allocation, priority, scheduling, capacity and delivery settings require restart. The 1200-byte packet limit remains explicit.

These defaults are development values, not validated physical capacity or optimal experimental tuning.

## Validation results

Eight relevant packages built successfully using ROS Jazzy with isolated build/install directories in `/tmp`: `surf_multirobot_msgs`, `surf_multirobot_comms`, `surf_drone`, `surf_humanoid`, `surf_data_tracker`, `bonxai_ros`, `surf_bringup`, and `surf_portable_bringup`. Final sender/receiver rebuild also passed after the numerical validation and broadcast recovery changes.

All **84 behavioral tests** pass:

| Area | Tests |
| --- | ---: |
| Sender allocation/debt, communication state and ROS delivery | 15 |
| Receiver ROS temporal/recovery integration | 1 |
| Codec | 3 |
| Tracker/capacity/identification/network | 33 |
| Hardware bringup/configuration/visualizer | 25 |
| Portable launch | 3 |
| Bonxai fusion policy | 4 |

Coverage includes priority factors/order, weighted sums versus counts, supersession, spatial reordering, stale DELTA/BACKLOG/retry rejection, class transfer, exact ACK debt resolution, deferred/failed outstanding debt, zero-debt/zero-capacity behavior, feedback response, shared-budget projection and actual CDR publication, overflow/invalid matrices, valid/invalid runtime updates, telemetry separation and tracker revisions. ROS integration also covers timeout/recovery without new scans and restoration of a cleared static-prior voxel. Bonxai fusion policy tests pass; a full physical-radio/Bonxai end-to-end deployment was not run.

The final six-package colcon test run passed in 23.7 seconds (`/tmp/surf-lqr-final-tests.log`). Bonxai's broader lint suite is not clean: flake8/pep257 failures in untouched launch files, existing uncrustify issues, and xmllint unable to retrieve the external ROS schema. Baseline formatting failures were reproduced from committed files; they are not counted as passing tests. See `/tmp/surf-lqr-results.txt` and `/tmp/surf-bonxai-baseline-style.log`.

Additional checks passed: main/nested `git diff --check`, Python compilation, actual ROS tracker callbacks into SQLite, and offline identification/figure generation. The synthetic identification smoke recovered its known A/B from 59 samples with rank 4, scaled condition 20.67, and approximately 3e-14 RMSE; this validates tooling, not the real robot model. Smoke artifacts are under `/tmp/surf-lqr-tracker-smoke` and `/tmp/surf-lqr-identification-smoke`.

## Remaining physical validation and technical risks

1. The enforced budget measures exact CDR bytes at ROS publication, with a bounded saved-credit burst. It is not a measured on-air byte/airtime guarantee. Bridge framing/compression, IP overhead, radio retransmissions, odometry and ACK/control traffic require capacity calibration or pacing at the final bridge/driver boundary before claiming a strict physical-link bound.
2. ACK confirms receiver validation, retention and local publication, not a separate successful Bonxai integration ACK. Retained local replay supports downstream recovery.
3. Timestamp monotonicity assumes coherent observation time within an epoch. Persistent per-coordinate history/tombstones use memory proportional to explored map size; pending-priority sorting costs O(N log N). Profile memory, control jitter and encoding on Jetson before field operation.
4. Projected LQR is not a globally optimal constrained controller or a stability proof under arbitrary capacity changes, matrix switching or ACK delays. Identification needs independent excitation and held-out validation.
5. Next experiments: calibrated load sweeps at fixed radio conditions with control overhead; induced loss/reordering and process restarts; both-class excitation without permanent saturation; held-out A/B validation; Q/R sweeps measuring debt, latency and backlog convergence. Rebuild both peers and Bonxai together because ROS message interfaces changed.

## Changed-file inventory

Paths below are relative to `surf_ws/src/distributed_mapping_surf`; `M` means modified, `A` added, `D` removed. The nested dependency contains two source changes; its parent gitlink has not been committed. The parent `SURF_2026/README.md` also documents the new link architecture. Pre-existing unrelated submodule changes were preserved.

```
 M README.md
 M config/robot.example.yaml
 M config/transport/drone.example.yaml
 M config/transport/humanoid.example.yaml
 M deploy/docker/.env.example
 M deploy/docker/compose.yaml
 M docs/INTEGRATION.md
 M docs/snapshot_recovery.md
 M docs/transport_and_state_fixes.md
 M surf_bringup/CMakeLists.txt
 M surf_bringup/config/drone_transport.yaml
 M surf_bringup/config/humanoid_transport.yaml
 M surf_bringup/launch/hardware.launch.py
 M surf_bringup/scripts/slam_map_visualizer.py
 M surf_data_tracker/README.md
 M surf_data_tracker/package.xml
 M surf_data_tracker/setup.py
 D surf_data_tracker/surf_data_tracker/adaptive_mode_report.py
 M surf_data_tracker/surf_data_tracker/analysis.py
 M surf_data_tracker/surf_data_tracker/network.py
 M surf_data_tracker/surf_data_tracker/tracker.py
 M surf_data_tracker/test/test_network.py
 M surf_drone/CMakeLists.txt
 M surf_drone/config/drone.yaml
 D surf_drone/include/surf_drone/adaptive_mode.hpp
 M surf_drone/include/surf_drone/communication_state.hpp
 M surf_drone/package.xml
 D surf_drone/src/adaptive_mode.cpp
 M surf_drone/src/drone_scan_sender.cpp
 D surf_drone/test/test_adaptive_mode.cpp
 M surf_drone/test/test_sync_supersession.cpp
 M surf_humanoid/src/drone_data_receiver.cpp
 M surf_humanoid/test/test_sync_recovery.cpp
 M surf_multirobot_comms/CMakeLists.txt
 M surf_multirobot_comms/include/surf_multirobot_comms/qos_profiles.hpp
 D surf_multirobot_comms/include/surf_multirobot_comms/snapshot_recovery.hpp
 D surf_multirobot_comms/test/test_snapshot_recovery.cpp
 M surf_multirobot_msgs/CMakeLists.txt
 M surf_multirobot_msgs/msg/CompressedVoxelDelta.msg
 M surf_multirobot_msgs/msg/DeliveryMetrics.msg
 M surf_multirobot_msgs/msg/LinkMetrics.msg
 M surf_multirobot_msgs/msg/RealtimeAckMetrics.msg
 M surf_multirobot_msgs/msg/SyncRequest.msg
 M surf_multirobot_msgs/msg/VoxelDelta.msg
 M surf_portable_bringup/launch/portable.launch.py
A  docs/information_allocation.md
A  surf_bringup/test/test_temporal_visualizer.py
A  surf_data_tracker/surf_data_tracker/capacity.py
A  surf_data_tracker/surf_data_tracker/identification.py
A  surf_data_tracker/test/test_capacity_identification.py
A  surf_drone/include/surf_drone/information_allocation.hpp
A  surf_drone/include/surf_drone/information_debt.hpp
A  surf_drone/src/information_allocation.cpp
A  surf_drone/test/test_information_allocation.cpp
A  surf_multirobot_comms/include/surf_multirobot_comms/temporal_voxels.hpp
A  surf_multirobot_msgs/msg/AllocationMetrics.msg
A  docs/implementation_report.md
M  dependencies/rolling_bonxai/bonxai_ros/include/bonxai_ros/bonxai_server.hpp
M  dependencies/rolling_bonxai/bonxai_ros/src/bonxai_server.cpp
```
