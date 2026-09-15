# Single-link information allocation

All robot-to-robot map data uses the approximately 1 GHz HaLow radio. DELTA
and BACKLOG are logical traffic classes competing for this same resource.
The old discrete adaptive controller and periodic whole-map radio snapshots
have been removed. Wi-Fi management/hotspot tools remain independent utilities.

The processing order is:

radio telemetry → usable application capacity → physical/map changes → temporal
validation and supersession → classification → priority-weighted debt → LQR →
shared capacity constraint → saved per-class byte budgets → priority selection →
compression and packetization → HaLow → receiver temporal validation → ACK → debt.

LQR decides **how much** communication each class requests. Priority decides
**which** currently valid observations to send. Neither can override temporal
validity. Priority can reorder different coordinates; it cannot reverse the
observation time of a coordinate.

## State and lifecycle

DELTA is information debt from a newly observed change in occupancy state:
static occupied, dynamic occupied, free, or deleted. First observations outside
the shared static prior are changes from unknown state. Re-observing unchanged
occupancy does not generate repeated debt. Static/dynamic classification changes,
ray clearing and dynamic expiry produce new evidence.

BACKLOG is outstanding information whose delivery was deferred or unsuccessful,
or whose previously acknowledged delivery knowledge became invalid after receiver
recovery. It is not a periodic transmission of the entire map. Recovery can
legitimately make every retained coordinate outstanding, but selection remains
incremental, prioritized, and capacity constrained.

One ledger entry per coordinate contains the latest changed state, observation
stamp, pending flag, traffic class, packet identity, creation time and priority.
A new change enters DELTA. Unsent DELTA moves to BACKLOG after
`delivery.defer_seconds`. A packet remains debt while in flight. Timeout clears
its packet association and makes it eligible for BACKLOG delivery. An ACK resolves
only entries still associated with that exact packet; a late ACK cannot resolve
a superseding observation. Resolved entries and deletion tombstones remain for
receiver recovery. No independent DELTA and BACKLOG copies are counted.

Newer changed observations replace older outstanding entries across both classes.
Their old weighted debt is removed as supersession, not successful delivery.
Selection, compression and publication never resolve debt. Packet attempts are
retired on ACK or timeout; timeout retries encode current candidates under a new
packet ID rather than retaining obsolete encoded generations.

## Temporal and recovery protocol

The existing SVD2 codec already carries per-record nanosecond observation times.
Sender scan stamps must increase; same-coordinate older/equal stamps cannot
replace ledger entries. Receiver `TemporalVoxels` uses one timestamp table across
both classes, retaining deletion timestamps. Incoming older/equal records are
rejected independently of packet version, priority or traffic class. Different
coordinates can arrive with arbitrary relative ages and packet versions.

Packet versions identify delivery attempts within a random sender map epoch.
They are not a global freshness cutoff for voxel state. Retired epochs cannot
reactivate during a receiver lifetime. Bonxai independently checks timestamps
on its expanded local voxels, including deleted-state timestamps, and rejects
retired epochs. New epochs intentionally reset the source layer.

Each normal packet is independently encoded and ACKed (`chunk_count=1`), avoiding
cross-packet assembly dependencies under spatial prioritization. Lost ACKs cause
reselection; stale retries are still ACKed. ACK means the receiver decoded,
validated, retained and published the accepted state locally; it is not a separate
Bonxai integration acknowledgment. The receiver retains accepted state and replays
it over **local DDS only** every five seconds for downstream restarts. This replay
never consumes radio bandwidth. On receiver startup a recovery request also works
when the sender has no new scans; retained entries become BACKLOG.

Old message/type/topic names `SyncAck`, `SyncRequest`, `RealtimeAck`, `/sync_tx`,
`/realtime_tx`, `full_refresh`, and adaptive constants remain compatibility artifacts.
Normal packets use `MODE_INFORMATION=4`, `full_refresh=false`, traffic 1=DELTA,
2=BACKLOG. Old adaptive/snapshot packets are rejected. Rebuild and upgrade both
peers and Bonxai together: added ROS message fields change interface types.

## Priorities and debt

For either class i:

```
p_i(v) = base_i
       + age_i * clamp(pending_age_seconds / age_seconds_i, 0, 1)
       + proximity_i / (1 + distance_to_humanoid_metres / distance_metres_i)
       + dynamic_i * [state == dynamic occupied]
       + destructive_i * [state == free or deleted]
```

Pending age starts when the changed observation enters the ledger; recovery
starts a new outstanding-delivery age for previously resolved entries. Proximity
is omitted without a fresh, common-map humanoid pose. No planning/path relevance
is used. Distances use voxel centers. Scores break ties by older observation time
and then coordinate for deterministic spatial order. DELTA defaults are
`base=age=proximity=dynamic=destructive=1`; BACKLOG age weight is 2. Both normalize
age by 10 s and distance by 10 m. Base must be positive, other weights nonnegative,
and normalization scales positive. These are explainable development values,
not experimentally validated priorities.

State ordering is `[delta_debt, backlog_debt]`, with `e_i = sum(p_i(v))` over
pending entries in class i, including in-flight entries. Raw pending counts are
recorded separately and are never substituted for weighted state.

## LQR and runtime tuning

Input ordering is `[delta_rate, backlog_rate]`, in serialized application bytes/s.
At nominal discrete timestep dt:

```
x[k+1] = A[k] x[k] + B[k] u[k] + d[k]
J = sum(x[k]' Q[k] x[k] + u[k]' R[k] u[k])
K[k] = LQR(A[k], B[k], Q[k], R[k])
u_requested = -K[k] x[k]
```

Outdoor-informed starting defaults at dt=0.1 s are A=I, B=-0.001 I,
Q=diag(100, 1), R=0.0001 I. See [outdoor tuning](outdoor_lqr_tuning.md) for
the three-run diagnosis, comparison, delivery windows, and validation limits.
Negative B explicitly models communication reducing debt; no absolute-value
feedback workaround is used. Q/R are control costs, not Kalman covariances.
The assumed B effectiveness requires identification with actual compression,
priority and delivery statistics. Changing dt requires corresponding model tuning.

`allocation.a`, `.b`, `.q`, `.r` each contain exactly four row-major doubles.
The controller solves the discrete algebraic Riccati equation by iteration using
Eigen LDLT, validates finite matrices, symmetric PSD Q, symmetric PD R,
convergence, finite K, and closed-loop spectral radius below one. The projected
physical system is nonlinear; this unconstrained stability check does not prove
stability under saturation or arbitrary time-varying models.

An atomic ROS parameter request can update all four matrices together. Validation
and gain calculation complete before the active model/gain/revision are replaced
under the state mutex. Invalid updates retain the previous controller and return
a parameter rejection. Near-identical matrices (1e-10 relative tolerance) do not
recompute K. Startup invalid matrix values retain the development controller;
wrong dimensions fail startup configuration. Other allocator/priority/scheduling
parameters require restart and are rejected if changed at runtime. Record every
revision before comparing controller experiments.

For example, an experiment client can call `set_parameters_atomically` with
four double-array parameters. Avoid four independent `ros2 param set` operations
when a coherent multi-matrix update is intended.

## Capacity and scheduling

`InformationAllocationController` does not estimate capacity. The separate
`halow_capacity_provider` uses the existing `NetworkSampler` abstraction.
Preference is a directly supplied application-capacity measurement, then a
guarded estimate from fresh MMRC rate-control telemetry and selected-rate success
probability. Achieved interface throughput never substitutes for capacity.
Missing or stale MMRC data grant zero map credit.
The sender gives link-metric callbacks their own executor group and lock so
controller/scan ledger work cannot delay a locally published measurement past
the three-second sender freshness timeout.

Source inspection of the installed Morse MM8108 2.0.0 driver found
`mmrc_table_csv` (selected MCS/bandwidth/GI, rate-control throughput statistics,
success/attempt and MPDU counters), `page_stats`, and mesh-only PHY/RSSI reporting.
The existing exporter exposes MMRC/page snapshots under `/run/surf-halow-telemetry`.
Exports older than three seconds are ignored. Linux interface counters, `iw`
station counters and valid RSSI are recorded when available. Generic Morse
nl80211 bitrates remain explicitly labeled compatibility values. MMRC average
throughput is a driver rate-control statistic, not measured application goodput
and not relabeled PHY rate. The live interface query in this development session
returned “No such device”; no live usable capacity or RSSI was established.

`experimental_mmrc_factor=0.1` reserves 90% provisional headroom; this is not a
measured application-goodput ratio. The factor multiplies MMRC Mbps by 125000
and selected-rate success probability to derive bytes/s. Calibration
must reserve room for return ACKs, odometry, reliable startup/recovery control,
bridge framing, IP/radio overhead and other shared traffic. `LinkMetrics` supplies
capacity with validity and method. Missing/stale/invalid telemetry makes map
capacity zero instead of silently granting an unmeasured development credit.

Projection clamps negative/nonfinite requested components to zero, then scales
positive rates proportionally if their sum exceeds usable capacity:

```
r_delta >= 0; r_backlog >= 0
r_delta + r_backlog <= b_usable
u = max(u_requested, 0) * min(1, b_usable / sum(max(u_requested, 0)))
```

Zero capacity clears saved credits. One wall timer runs independently of scans;
actual dt and computation timing are logged. Class scheduling frequencies default
to 10 Hz; their opportunities cannot exceed the controller frequency. Allocated
rate accrues byte credits, equivalent to rate/frequency bytes per opportunity.
Each class credit and the shared capacity credit are bounded by two nominal
control intervals of their respective rate, with a one-packet minimum. This
permits a bounded saved-credit burst; packets are not continuous fluid. At the
development defaults the shared ceiling is 2000 bytes. A callback may publish
multiple independently ACKed packets while debiting these same credits. Work is
bounded to 16 packets per class per callback and 2400 candidates at the default
1200-byte packet limit. Combined cumulative publication is bounded by integrated shared
capacity plus credit saved before a measurement window. Executor catch-up is
bounded to two nominal intervals.

Candidate selection partitions out a bounded highest-priority prefix before
sorting that prefix, rather than sorting the entire pending map. Bounded trial
encoding searches prefix sizes and retains the largest explicitly verified fit
within both credits and the hard 1200-byte packet limit. Nonmonotone compression
can make this search miss a larger feasible prefix, but cannot make a packet
exceed its budget. Uncompressed voxel size never substitutes for wire cost. Both classes debit the same shared credit. No local encoded retry
queue exists; downstream bridge FIFO may still contain already offered old
packets, which the receiver safely rejects.

Routine control work iterates an active pending-coordinate index. Resolved
coordinate history remains available for receiver recovery without increasing
routine controller work. Timeout/defer transitions, scoring, age/state metrics,
and per-class candidate collection share one active-set traversal reported as
`debt_cycle_ms`; bounded ordering remains timed per class in `selection_ms`.
Legacy advance, score, and metric-scan timings are zero for the fused controller.
Telemetry also records active and retained counts, pending age buckets, selected
pending-age mean/max, old-entry selection counts, and selected occupancy states.

To prevent permanent starvation after the normal priority age term saturates,
each class reserves a configurable fraction of an estimated packet-sized
candidate window for entries older than `scheduling.starvation_age_seconds`.
Reserved entries are oldest-first; all remaining candidates keep priority order.
Defaults are 10 percent and 30 seconds. The reserve remains subject to the class
allocation, shared credits, packet limit, temporal checks, and ACK accounting.

The data bridge is UDP on HaLow for both map topics. A second **logical** TCP
bridge, also addressed over the HaLow subnet, preserves reliable recovery/control
and initial-pose delivery. It has no map bandwidth budget. The configured usable
map capacity must exclude that traffic. CDR bytes are offered ROS application
bytes, not measured on-air bytes: a strict physical airtime limit would require
pacing/instrumentation at the driver or final bridge boundary.

Both ACK topics use bounded 256-message/65536-byte bridge FIFOs. This matters
when one controller step emits multiple packets: reliable TCP cannot recover an
ACK overwritten before the bridge timer consumes it. FIFO overflow rejects the
newest ACK with a warning instead of silently replacing an older packet ID.

## Experiment outputs and identification

`AllocationMetrics` is recorded at every control step in the existing per-host
SQLite event store and aggregated by `analyze_run`. It contains counts, weighted
state, cumulative generated/acknowledged/superseded/reclassified/timed-out debt,
net disturbance, model/gain matrices, revision/update timestamp, requested and
allocated rates, capacity/method, projection scale, targets, selected records,
priority totals/maxima, serialized/payload/uncompressed bytes and timing.
Disturbance includes generated/recovery debt, supersession, class transfer and
priority drift and peer-mask exclusion; it excludes successful ACK removal. Difference cumulative events
between consecutive steps. `previous_debt` is the preceding sampled state;
the next row supplies the next state. Receiver metrics record stale rejection and
zero applied temporal regressions. ACK metrics retain same-clock RTT and timeout
observations for both classes.

```
ros2 run surf_data_tracker analyze_run /path/to/run
ros2 run surf_data_tracker identify_allocation /path/to/run
```

Identification writes `allocation_steps.csv`, `allocation_identification.json`
`allocation.png` and `allocation_diagnostics.png`, not per-step files. It excludes epoch/revision boundaries,
missing steps and >20% timing deviations, and fits
`[A B] = least_squares([x u], x_next - d)` with column scaling. Output includes
sample count, rank, scaled conditioning, residual RMSE and explicit model-validity
cautions. The figure compares debt, per-class allocation/capacity and offered
load. Existing wireless plots/timeseries retain radio state, interface throughput,
loss/retries and pose alignment for calibration. No online A/B adaptation or
unsupported distance law is fitted.

Next experiments should sweep offered load at fixed radio conditions to calibrate
capacity with control overhead; inject loss/reordering/restarts; excite both debt
classes while avoiding continuous saturation; then vary Q/R and compare debt,
latency, bytes, starvation and backlog convergence. ACK delay adds unmodeled
state; a two-state fitted model must be checked against held-out experiments.
Indefinite temporal/deletion and receiver-recovery caches consume memory
proportional to explored coordinates; safe compaction needs an explicit protocol,
not a time-based deletion that permits stale resurrection.

See [indoor-test fixes](indoor_test_fixes.md) for scheduling, radio caching,
receiver observation-age metrics and experiment provenance added after the
first indoor run.

Map DDS publishers use volatile durability so a bridge reconnect cannot replay
unbudgeted historical packets; outstanding ledger debt supplies paced recovery.

The tracker records projection/saturation flags and per-class offered CDR rates.
New-protocol packet-loss inference uses the combined DELTA/BACKLOG attempt
sequence; normal interleaving is not mistaken for dropped packets. Temporal
regression metrics also appear in summary output, and a nonzero applied
regression makes analysis fail explicitly. Missing interface throughput remains
unavailable rather than being reported as a measured zero.

Recovery startup requests use an empty source ID and epoch zero because a
restarted receiver may not know the configured sender name. Normal directed
requests still name their source. Portable Docker defaults to host-only DDS
discovery, matching the Jetson deployment, so direct DDS traffic cannot bypass
the radio bridge scheduler. If a robot needs an internal multi-host ROS graph,
configure discovery explicitly and isolate that graph from the other robot.

Telemetry additionally records both oldest/newest pending observation ages in
the sensor/ROS clock domain, the maximum lag from previously acknowledged
observations where known, the number of lag samples, discarded superseded
pending entries, and the actual per-class codec. A cleared shared-prior voxel
is explicitly restored by DELTA when reoccupied; no radio snapshot is needed.
