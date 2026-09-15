# Observed free space and bounded regional delivery

The tuned outdoor trials delivered many more DELTA packets but still accumulated
846k–967k pending BACKLOG voxels. The sender only cleared previously occupied
cells; it did not communicate most newly observed free space. Dynamic expiry
also incorrectly served as free-space evidence. These changes address those
semantics and the map-size-dependent scheduling cost.

## Evidence semantics

- Measured rays generate `STATE_FREE` for traversed, previously unknown cells.
  Existing occupied cells (including shared-prior occupancy) require
  `filters.clear_min_misses` observations before clearing.
- Rays use the measured endpoint and sensor origin. Voxel traversal is contiguous,
  excludes the endpoint, stops at current-scan occupied cells and the peer mask,
  and respects the existing height, near-range, and self filters. A work limit
  truncates a ray prefix rather than stepping over cells. Edge/corner-only
  contacts do not generate extra side-cell evidence.
- Repeated unchanged free observations are deduplicated. Occupancy observed
  later supersedes free evidence with a newer timestamp.
- Expired dynamic occupancy produces `STATE_UNKNOWN=5`: withdraw this source's
  evidence, retain temporal ordering, and do not claim the cell is free. UNKNOWN
  neither erases another source's obstacle nor supplies evidence to clear the
  shared prior. Reobservation can subsequently establish occupied or free state.
- Legacy `STATE_DELETE=4` retains its existing scan-clearing/free semantics.
  New ray clears use FREE for both static and dynamic cells.
- Bonxai applies equal-resolution coordinates directly. Float32 representation
  of `0.05` must not expand an update into adjacent 5 cm cells. Genuine differing
  resolutions still use the existing cell-expansion path.

Free evidence is limited to observed ray cells. Unsampled space stays unknown;
there is no interpolation that fills a block merely because some cells arrived.
The communication representation remains 5 cm by default. Increasing its voxel
size changes what a single beam represents and requires separate validation.

## Work limits and scheduling

The defaults are a sensor-centered 10 m communication radius,
`mapping.maximum_ray_cells_per_scan=32768`, and
`scheduling.maximum_entries_per_cycle=4096`. These parameters are in `drone.yaml`
and also declared as node defaults. Endpoints outside the communication radius
can contribute the observed ray prefix inside the radius, but their occupied
endpoints are not inserted into the communication ledger. Existing retained
history is preserved for delivery and recovery when the sensor moves onward.
The radius therefore bounds new spatial coverage per scan, not total memory.

The ray count and per-ray limits remain additional upper bounds. A rotating,
sorted endpoint cursor prevents the total work budget from always taking the
same point-cloud prefix. Ledger commits release the state lock every 256 records.
Metadata is installed before the first batch becomes visible to control.

Control uses bounded shares of work for fresh records, a focused spatial block,
and round-robin history. Outstanding counts and cached weighted debt remain
**global**, including in-flight debt; they are not extrapolated from the sample.
Age/proximity scores and defer transitions are refreshed on visited entries.
Consequently scoring/classification can lag under sustained input overload;
this is an incremental scheduler, not an exact instantaneous full-map rescore.
Receiver recovery is enumerated in bounded slices rather than under one long lock.

The LQR matrices and shared byte-credit ceiling retain their tuned values.
Before capacity projection, streams with no eligible sampled candidates have
zero serviceable request. This prevents in-flight or unvisited debt from reserving
bandwidth that cannot be spent in that cycle. Telemetry retains both the raw
LQR request and the serviceable request. Obstacle observations have an explicit
`priority.{delta,backlog}.occupied=2` bonus so new free-space volume does not outrank
obstacle evidence at equal age/proximity. These are scheduling policy changes;
previous closed-loop fits should not be reused as validation of the new behavior.

ACK callbacks have a separate callback group and the standalone sender uses a
two-thread executor. `ack_lock_wait_ms` measures callback-entry-to-lock delay;
it does not measure pre-callback DDS/executor delay. Packet encoding and control
still use the state mutex, but full-ledger traversal no longer holds it.

## Spatial progress

Both streams rank individual voxels using their existing priority plus
`scheduling.spatial_change_weight * log(1 + changing_neighbors)` (default weight
1.0). Neighbors are pending, unsent, non-UNKNOWN observations in the configured
proximity neighborhood, including observations outside the current scoring sample.
Delivered history does not inflate the score. This favors dense local changes
without requiring an atomic cluster message or suppressing isolated changes.
The existing age reserve precedes this score. The bonus affects selection only;
information-debt accounting retains its original units.

BACKLOG groups output by a bounded voxel-proximity component. The default
neighborhood includes all 26 immediately adjacent cells and axial gaps up to two
voxels (10 cm at 5 cm resolution), which keeps sparse, quantized samples in the
same cluster without an expensive 5×5×5 search per voxel. A controller cycle may
service several components to avoid wasting capacity on small clusters. Records
are ordered component-by-component; a packet may contain several small complete
component groups when keeping them separate would waste most of its byte budget. The search
starts from receiver-acknowledged evidence when possible, then expands through
nearby FREE and occupied records. It may cross acknowledged records to
reach the current pending frontier. UNKNOWN records break the graph. Proximity
clustering is a communication grouping rule, not a claim that diagonal contact
or a gap is traversable; planner clearance still requires a later map-layer rule.

The component search is bounded by
`scheduling.maximum_cluster_traversal_entries=8192`, with neighborhood radius
`scheduling.cluster_neighbour_radius_voxels=2`. If no acknowledged anchor
exists, the highest-ranked sampled record seeds a new component. Focus is
reconsidered each cycle, including after a truncated search, so new hotspots and
age-reserved updates can displace an earlier focus. Component
selection does not assert traversability: occupied voxels are included as boundary
evidence, and no robot clearance or ground-support model exists yet. DELTA keeps
per-voxel spatial priority order so a new obstacle is not delayed behind regional completion.
The deployment profile enables this with `scheduling.cluster_enabled=true`.
Per-voxel observation timestamps, supersession and packet ACKs remain authoritative;
no new atomic snapshot/manifest protocol is introduced.

Allocation telemetry reports the focused block key, its revision, number of
known records, outstanding records and a cumulative count of acknowledged block
completions. A revision increments for new information/recovery activation.
This is **delivery of known observations**, not proof that all 4096 cells were
observed, downstream Bonxai finished integrating, or a planner verified clearance.
`recovery_entries_remaining` must be zero before interpreting drainage as receiver
reconstruction completion. Completion can be followed by a newer revision.

`sampled_age_metrics=true` explicitly marks age buckets, age/priority maxima and
ACK-lag statistics as sampled. `scored_entries` gives the sample size. Do not
compare those extrema directly with the old exhaustive outdoor telemetry.
Pending/in-flight counts and debt remain global. `selected_unknown_count` is
separate from the legacy four-state arrays. Pipeline telemetry records new FREE
and UNKNOWN counts, sampled rays, and visited ray cells.

`focus_cluster_seed`, `focus_cluster_visited` and
`focus_cluster_candidates` describe the bounded connected BACKLOG search.
`completed_cluster_deliveries` counts searches whose focused component had no
eligible records. It does not prove the physical region was completely observed,
that free cells have robot clearance, or that a downstream planner can use it.

## Validation and deployment

Rebuild the message package and its consumers together on both hosts, including
sender, receiver, Bonxai, visualizer and tracker. UNKNOWN extends voxel semantics;
old map consumers do not understand it. Changed telemetry messages also require
matching generated interfaces. Local cluster validation used
`/tmp/surf-cluster-install`; these changes do not deploy robot images.

The regression tests exercise contiguous rays and truncation, occlusion,
previously unknown free-space delivery, occupied endpoint protection, expiry,
recovery/supersession, region progress, debt conservation, bounded traversal,
codec round trips and Bonxai cross-source temporal fusion. The large-debt test
retains its 50 m synthetic fixture radius explicitly, independent of the new
10 m deployment default, and checks byte budgets and offered throughput.

The standalone benchmark is `surf_drone/test/benchmark_information_debt.cpp`:

```sh
g++ -std=c++17 -O2 \
  -I surf_ws/src/distributed_mapping_surf/surf_drone/include \
  -I /opt/ros/jazzy/include/tf2 \
  surf_ws/src/distributed_mapping_surf/surf_drone/test/benchmark_information_debt.cpp \
  -o /tmp/benchmark_information_debt
/tmp/benchmark_information_debt
```

Run a stationary bounded scene next. Compare the sender-local and receiver maps
with unknown cells blocked, check actual free-cell coverage and obstacle
clearance, then measure time until a connected planning region is usable.
Observe ACK lock wait, controller p95, serviceable/offered bytes, regional drainage
and the effect of the ray budget. Repeat on the Orin and radio: a workstation
benchmark is not a field throughput or planner-readiness result.

## Local results (2026-09-14)

Release builds of the message, codec, sender, receiver and Bonxai packages passed.
All 42 C++ test cases passed: 33 sender/allocation/ray/debt tests, 3 codec tests,
1 receiver test, and 5 map/fusion tests. The new tracker fields also passed a
serialization smoke check against the rebuilt generated messages; both changed
Python modules compile. ROS tests exercised local same-process delivery despite
sandbox network socket restrictions; they are not a radio test.

The 60k-voxel no-ACK fixture disables cluster ordering to isolate byte-credit
behavior; the final run offered 9.53 kB/s against its 10 kB/s budget while
respecting per-packet, per-class and shared credit limits. Its denominator is
retained scheduling time (the fixture's `active_time`), not a field goodput rate.

On this workstation, a one-million-pending-entry full traversal took 917 ms.
Bounded scoring plus ranking over 100 cycles had 5.96 ms median / 6.58 ms p95.
At 100k entries the corresponding values were 31.2 ms full traversal and
1.82 / 2.22 ms bounded median / p95. These measure ledger work, not complete
scan processing, packet encoding, radio throughput, or Orin performance.
