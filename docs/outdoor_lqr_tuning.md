# Outdoor LQR diagnosis and starting parameters

Reviewed `experiments/lqr_outdoor`, `lqr_outdoor_2`, and `lqr_outdoor_3` using
both hosts' SQLite allocation, delivery, ACK, pipeline, and configuration events.
These are three runs of the same configuration, not a before/after experiment.
All three sender binary hashes match
`99f8f6ceaf5bdfa907d3e34217e62f8dbde6ec38f0b451e08ebcdbf5afa88c11`.
The changes below are starting parameters for the next outdoor validation;
offline comparisons do not establish improved closed-loop performance.

## Measurements

Statistics below use active controller samples unless stated otherwise.
Rates are offered serialized application bytes, not on-air capacity or goodput.

| Measurement | outdoor | outdoor_2 | outdoor_3 |
|---|---:|---:|---:|
| Allocation samples (active) | 1142 (895) | 1027 (851) | 1060 (956) |
| Final BACKLOG voxels | 804,919 | 568,336 | 975,838 |
| Control interval median / p95, ms | 225 / 780 | 178 / 466 | 300 / 892 |
| Control computation median / p95, ms | 213 / 762 | 168 / 454 | 288 / 874 |
| Debt traversal median / p95, ms | 178 / 569 | 139 / 347 | 244 / 664 |
| Intervals at least old 200 ms defer window | 52.3% | 40.1% | 54.5% |
| Offered bytes / active second | 4,948 | 7,119 | 4,188 |
| Oldest pending age maximum, s | 263.5 | 158.4 | 351.9 |
| ACK successes / timeouts | 696 / 861 | 700 / 754 | 637 / 1055 |
| Timed-out packet IDs also at receiver | 857 | 505 | 1047 |
| Successful ACK RTT p99 / max, ms | 2466 / 2638 | 2207 / 2264 | 2428 / 2487 |
| Scan clearing median, ms | 303 | 264 | 323 |
| Median static-prior overlap per scan | 0.53% | 0% | 0.49% |

Every active sample is capacity saturated. With equal diagonal gains, proportional
projection gives DELTA only 1.30%, 1.48%, and 1.37% median allocation when both
classes contain debt. DELTA emitted only 14, 12, and 18 packets recorded by the
receiver. The old 0.2 s defer window can move fresh unsent debt into BACKLOG before
the next overloaded control callback selects it. A larger queue then increases
the full active-ledger traversal cost, worsening callback delays.

The scheduler deliberately retains at most two nominal intervals of credit.
Only 49.9%, 71.9%, and 42.2% of elapsed active time survives that clamp, closely
matching observed use of the 10,000 byte/s budget. Increasing the configured
radio capacity alone will not repair missed scheduling time. The LQR arithmetic
is not the expensive traversal, ranking, or scan clearing work.

ACK timeout is not a forward-loss estimate: 99.5% and 99.2% of timed-out packet
IDs in runs 1 and 3 are present in receiver delivery events. This supports a
return-path/executor-delay diagnosis, but does not identify exactly where each
ACK was delayed or lost. Same-clock sender RTT includes callback and lock delays;
successful RTT samples are censored by timeout handling. Run 2's receiver
recording is shorter than the sender recording, so its unmatched IDs cannot
all be attributed to loss. Packet matching uses epoch and version, without
depending on cross-host clock alignment. No receiver event reports an applied
temporal regression. Stale voxel rejection totals are 146,703 / 61,864 / 183,917;
these include obsolete or repeated observations, not proof of map corruption.

## Applied starting configuration

| Parameter | Before | After | Reason |
|---|---|---|---|
| `allocation.q` | diag(1, 1) | diag(100, 1) | Increase fresh-debt service while retaining BACKLOG tail gain |
| `delivery.defer_seconds` | 0.2 in YAML, 0.1 bare-node default | 1.5 | Exceeds the worst observed 1.001 s controller interval with margin |
| `delivery.ack_timeout_seconds` | 2 | 4 | Exceeds observed successful RTT and allows executor-delay margin |
| `maximum_clear_rays` in deployment YAML | 4096 | 1024 | Reduce costly ray work using the existing rotating subsampling |

The controller's compiled Q and delivery defaults match the deployment values.
Bare-node clear-ray fallback remains its existing conservative 256; the deployed
outdoor profile explicitly selects 1024. Ray length, range, voxel resolution,
miss/hit thresholds, and temporal validity rules retain their existing values.
Reduced ray coverage may delay free-space/deletion evidence; measure that delay
in the next run. A fourfold ray cap reduction is not a measured fourfold speedup.
The longer timeout also delays genuine-loss retry. The longer DELTA window is
not a guarantee of service under indefinite saturation.

In-flight entries remain part of weighted debt by design. With long ACK delays,
DELTA can have allocation but no eligible unsent candidates, leaving some shared
capacity unused. A synthetic no-ACK, 10 s timeout fixture with the new DELTA
window offered about 6.45 kB/s over its short observation window. This is a real
tradeoff, not a predicted field improvement. The existing no-ACK throughput test
now explicitly sets its original 0.1 s deferral to isolate BACKLOG packet-budget
behavior; it does not validate throughput with the outdoor delivery windows.
The added outdoor tests cover gain balance, small BACKLOG tail allocation,
zero-capacity behavior, delayed ACK acceptance, deferral, and eventual timeout.
Measure offered/allocated efficiency and in-flight debt in the next field run.

Keep `dt=0.1`, A=I, B=-0.001 I, R=0.0001 I, the 10 Hz scheduling opportunities,
10,000 byte/s development capacity, and disabled MMRC conversion. Tuned feedback
magnitudes are approximately [618.034, 95.125], versus [95.125, 95.125]. Nominal
unconstrained poles are approximately [0.382, 0.905]. Shared projection and saved
credit limits still bound publication; these poles do not prove constrained
system stability.

Frozen-state evaluation on each run raises median DELTA allocation when both
classes are pending to approximately 7.9%, 8.9%, and 8.3%. This compares requests
on the *recorded* debt states; it does not predict new debt, packet sizes, ACKs,
or gains from the longer deferral window. Q=1000 has diminishing gain benefit.
An alternative R_BACKLOG=1 produced much larger DELTA shares, but reduces isolated
BACKLOG gain about 95-fold and risks long small-queue packet accumulation delays;
it was rejected. Increasing DELTA Q leaves isolated BACKLOG allocation intact.

## Initial conditions and model limits

The first recorded samples (steps 37, 7, and 12) all have zero debt. Keep initial
debt and byte credit zero; do not preload artificial debt to force transmission.
Receiver recovery must continue to derive outstanding debt from the actual retained
ledger. Never discard or mark the large recorded BACKLOG acknowledged to reset a
run's state. Use a fresh epoch for a fresh experiment and preserve the normal
receiver recovery protocol.

Prior overlap is extremely low. These runs behave largely like mapping unknown
space rather than communicating small changes to a shared prior. This may reflect
new outdoor coverage, prior coverage/resolution, or alignment; telemetry cannot
distinguish them or determine a corrected initial x/y/yaw. Before the next run,
load the same intended map on both robots, establish localization, and check that
known surfaces overlap the communication prior. No geometric pose offset or
map-resolution change is justified by these data alone.

Exploratory fits retain only 234 / 246 / 227 active consecutive pairs within 20%
of nominal dt, biasing them toward early, smaller queues. Although rank is four
and scaled condition about 5.7–5.9, fitted B_DELTA varies from -0.022 to -0.118;
some off-diagonal coefficients imply communication creates debt. BACKLOG residual
RMSE is 620–650 priority units. Independent excitation, ACK-delay state and held-out
prediction validation are absent. Do not install these fits as a calibrated B.
Likewise, all capacity events use the development fallback: neither MMRC nor
achieved throughput establishes available application capacity.

These parameters mitigate the observed timing mismatch and allocation imbalance;
they do not remove O(pending-map-size) controller traversal or prove that an
expanding outdoor map can be drained over this link. Further controller work must
bound active-ledger scoring/ranking time without breaking debt accounting. Simply
lowering nominal frequency changes the discrete model and lowers service
opportunities; simply allowing unlimited catch-up creates radio bursts.

## Reproduction and next-run validation

From the repository root:

```sh
MPLCONFIGDIR=/tmp/surf-matplotlib python3 \
  surf_ws/src/distributed_mapping_surf/surf_data_tracker/scripts/review_outdoor_lqr.py \
  experiments --output experiments/lqr_outdoor_review
```

This produces `summary.json` and `outdoor_lqr.png`, opens raw databases read-only,
and records the rejected exploratory fits and parameter comparisons. NumPy and
Matplotlib are required. The figure shows debt accumulation, timing mismatch,
and allocation comparison, separately from any claimed performance prediction.

For the next outdoor experiment, record the effective parameter snapshot and
runtime artifacts, confirm the shared prior before motion, repeat a comparable
route, then stop moving and observe a stationary drainage period. Check actual
control p95 against 100 ms, successful DELTA packet rate/observation age, ACK
timeouts matched to receiver IDs, accepted clearing/deletion age, and whether
BACKLOG declines during the stationary period. Retain per-class starvation metrics.
If callbacks still exceed 100 ms, prioritize bounded ledger work before further
matrix fitting. Calibrate usable capacity separately with controlled offered-load
sweeps and return/control traffic included.

Local validation: ROS Jazzy Release build succeeded in `/tmp/surf-outdoor-build`
with installation isolated in `/tmp/surf-outdoor-install`. All 22 sender tests
pass (14 allocation/debt/temporal, 5 communication-state, 3 ROS delivery tests).
The unchanged communication-state suite passed on the first run; the two affected
suites passed after fixing a Riccati-tolerance assertion (about 1e-6 byte/s numerical
difference) and making the no-ACK fixture's original defer window explicit.
The ROS tests exercised local same-process communication; sandbox UDP socket
restrictions mean this is not a network/radio validation. Analysis completed on
all six raw databases, the generated plot was inspected, YAML parsed successfully,
and Python compilation and git whitespace checks passed. No robot deployment or
new outdoor trial was performed.
