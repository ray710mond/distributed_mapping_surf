# Chunk delivery and communication-state scaling

## Findings

The outdoor recording alone cannot locate each missing chunk. Source inspection
and a local regression do establish two software loss risks:

* Upstream network_bridge 3.0.0 stores a single latest serialized message per
  topic. Multiple callbacks between send ticks overwrite earlier chunks. The
  regression queues six callbacks before servicing a send: upstream behavior
  yields only chunk 5, even without a radio.
* Realtime local DDS history was two samples, smaller than a six-chunk update.
  Executor stalls could evict chunks at either local ROS hop. This is separate
  from UDP loss on HaLow.

Both mechanisms can worsen under CPU load. This does not establish that they
account for every missing chunk in outdoor_test_1; repeat hardware telemetry
is needed to measure remaining RF/driver loss.

## Transport changes

The workspace vendors network_bridge 3.0.0 (MIT), pinned to upstream commit
`b58c0fe19cbb52d1b3ba8fe3b6873f82124d767c`, with opt-in per-topic FIFO forwarding.
Default latest-value state forwarding is unchanged. Configured chunk topics:

| Topic | Maximum queued messages | Maximum queued serialized bytes |
|---|---:|---:|
| `/realtime_tx` | 256 | 307,200 |
| `/sync_tx` | 256 | 16,777,216 |

Parameters are `<topic>.queue_depth` and `<topic>.queue_bytes`. Both limits
apply; an oversized message or a full queue rejects the newest message and
logs a cumulative drop count. This bounds backlog across a disconnection and
makes overflow explicit. A queued chunk is never overwritten by a newer chunk,
and queued topics never replay stale data. The timer still forwards at the
configured rate (100 Hz here). UDP remains best effort over the air; this is
not a new end-to-end retransmission protocol.

Local chunk publishers/subscribers use reliable DDS with 256 samples of
history. The receiving bridge also retains 256 samples for
`CompressedVoxelDelta`. Reliable local DDS does not change HaLow UDP to TCP.
Sync still uses the existing retry protocol and 5 GHz TCP link. These changes
are not the proposed single-network architecture or a repair of every possible
TCP reconnection failure.

Build and source the workspace on both hosts, including `network_bridge`,
`surf_multirobot_comms`, `surf_drone`, `surf_humanoid`, and `surf_bringup`.
The normal full-workspace build and Jetson image include the vendored package.
Verify `ros2 pkg prefix network_bridge` resolves to the workspace install
(or `/opt/surf` in the rebuilt container), rather than `/opt/ros/jazzy`.
Do not run the new configuration with only the stock binary bridge installed.

## State-processing changes

* Static prior: immutable shared ownership replaces a complete hash-set copy
  per scan; the mutex now protects only publication/acquisition of the pointer.
* Humanoid mask: a spatial block index retrieves nearby cells and tombstones.
  A world-axis bounding box encloses the rotated mask, followed by the original
  exact voxel-center containment test. It no longer scans all accumulated state.
* Dynamic expiry: one scheduled expiry per dynamic coordinate replaces the
  all-cell scan. Observations reschedule it, promotion/masking/ray clearing
  cancel it, and due entries emit the same free records. The original strict
  `version - last_seen_version > retention` boundary is preserved.

Full snapshots necessarily still enumerate the map. Sampled ray clearing is
unchanged; reduced map-maintenance costs should be measured before weakening
clearing coverage. Spatial indexes cost extra memory proportional to stored
state; expiry memory is bounded by currently tracked dynamic coordinates.

## Validation

The local build of the four affected C++ packages succeeded. All 16 behavioral
test cases passed, together with the enabled formatting/XML/CMake checks. The
aggregate colcon report has zero errors/failures; 14 cppcheck entries were skipped.
`px4_msgs` was unavailable locally, so the unrelated optional PX4 bridge was not
built. Runtime code has not been deployed to the Jetson.

Tests include a 180-chunk burst through ROS and loopback UDP, legacy overwrite
reproduction, FIFO order and byte/depth overflow, randomized spatial mutation
queries, negative block boundaries, rotated mask equivalence, and dynamic
expiry equivalence across observation/cancellation sequences. Codec and adaptive
controller regression tests are also retained.

`surf_drone/test/benchmark_communication_state.cpp` is a reproducible synthetic
query benchmark. One laptop run at one million cells measured approximately
136 ms for a full scan and 0.13 ms for indexed querying, with identical results.
Reproduce from the repository root with:

```bash
g++ -O2 -std=c++17 \
  -I surf_ws/src/distributed_mapping_surf/surf_drone/include \
  -I /opt/ros/jazzy/include/tf2 \
  surf_ws/src/distributed_mapping_surf/surf_drone/test/benchmark_communication_state.cpp \
  -o /tmp/surf-state-benchmark
/tmp/surf-state-benchmark
```

This is not an end-to-end Jetson benchmark; incoming-scan processing, ray
clearing, compression, and snapshot generation remain additional work.

After deployment, repeat the same excursion and compare receiver chunk indices,
completion rate, selection/clearing times, and any bridge queue-overflow logs.
The field result is pending; a loss-free loopback test does not establish
loss-free outdoor operation.
