# Fixes from LQR_indoor_test

The indoor run delivered all 479 map packets but offered about 5.1 kB/s against a 10 kB/s allocation and ended with 50,615 pending BACKLOG voxels. The following changes address the implementation issues; capacity calibration and A/B/Q/R tuning still require a new experiment.

## Sender

- Highest-priority selection now partitions the candidate set and sorts only a bounded prefix. At the default packet limit, at most 2400 candidates per class are retained for scheduling; priority order and same-coordinate temporal rules are unchanged.
- Each packet attempt retains its coordinates, so an ACK touches only its own entries. Entries superseded since that attempt cannot be resolved by its ACK.
- The scan worker performs transform lookup, preprocessing and raw serialization outside the debt lock. Ray tracing, clearing and expiry operate on worker-owned map structures outside that lock as well. Observations are committed to the debt ledger under the lock. Control/ACK callbacks can therefore run during expensive ray processing.
- Shared and per-class saved credits are capped at `max(packet_limit, respective_rate * 2 * nominal_dt)`. Zero capacity clears credits; catch-up remains capped at two nominal intervals. At the default 0.1-second timestep and 10 kB/s capacity, the shared credit ceiling is 2000 bytes.
- A callback may send up to 16 independently encoded packets per class while debiting the same shared/class credits. Every packet still fits the hard 1200-byte limit. Per-step telemetry sums all packets rather than overwriting earlier packet counts/bytes.
- Prefix fitting uses at most 12 encoded trials per packet and retains the largest checked fitting candidate. Nonmonotone compression may mean a larger feasible prefix is missed; it cannot cause overspending because every selected packet's CDR size was explicitly checked.

These bounds permit a finite saved-credit burst. They do not claim continuous physical airtime pacing. Persistent severe executor stalls or rates beyond the bounded packet/candidate work limit can still cause underutilization and should remain visible in telemetry.

## Radio telemetry

`NetworkSampler` retains successful MMRC samples between rate-limited polls and reports measurement age, sample validity and poll status. Cached values expire after three seconds, including the exported file's age; reuse does not reset that age. A failed read may use still-fresh prior data, but cannot keep it alive indefinitely. This prevents ordinary 1 Hz timer jitter from making the capacity provider alternate between MMRC conversion and fallback.

Linux interface operstate is preserved separately from station association status, including AP-style `iw link` output. Large disagreement between instantaneous and average RSSI is flagged as inconsistent; raw in-range readings remain available for diagnosis. No new bandwidth efficiency factor or RSSI-to-distance model was introduced.

## Receiver and experiment data

- Receiver decode/reconstruction timing is measured. Unavailable downstream integration and transport-only timing use NaN rather than a fabricated zero.
- `DeliveryMetrics` adds accepted voxel count and oldest/newest/mean accepted observation ages in seconds. These come from accepted per-voxel stamps, not the latest scan header. An all-stale packet has zero accepted records and unavailable age statistics. Negative ages are retained to expose clock problems.
- Tracker records the new fields and captures initial parameters for discovered sender, receiver, capacity and bridge nodes, plus subsequent parameter events. Matrix revision telemetry remains unchanged.
- Run metadata includes installed sender/receiver/tracker artifact hashes, tracker Python implementation hashes and available configuration hashes. Compose provides the image reference; optional `SURF_IMAGE_DIGEST` records an operator-supplied digest, otherwise explicitly `unavailable`. The digest is not inferred from an image tag.
- Analysis flags one-way latency smaller than the recorded combined clock uncertainty. Observation-age plots/statistics require the existing verified-clock condition.
- Cumulative allocation counter totals use observed changes independently by host/source/epoch. Signed disturbance changes are preserved; unknown initial counter values are excluded. Timeseries no longer label sums of cumulative snapshots as totals. Existing network counter delta handling remains in place.

The original indoor SQLite data and primary analysis outputs were preserved. A separate reanalysis under `/tmp/surf-indoor-reanalysis` verified the corrected counter totals and uncertainty flags.

## Deployment and next experiment

Rebuild both peers because `DeliveryMetrics` changed. Standard Docker deployment requires rebuilt images and recreated containers; the Jetson development overlay can use a ROS workspace rebuild. These changes have not been deployed by this implementation task.

Leave development capacity and LQR matrices unchanged until testing the scheduler fixes. Repeat the indoor run with a stationary tail long enough to observe backlog drainage. Compare actual dt, priority/clearing/ACK timing, offered bytes against allocated bytes, oldest pending and delivered observation age, and MMRC validity. Then run controlled capacity sweeps and tune A/B/Q/R from adequately excited, independently validated data.

## Validation

All eight affected/dependent packages built successfully in the isolated ROS Jazzy `/tmp/surf-lqr-build` and `/tmp/surf-lqr-install` workspace. All 91 behavioral tests pass: sender 17, receiver 1, codec 3, tracker 38, hardware bringup 25, portable bringup 3, and Bonxai fusion policy 4. Existing unrelated Bonxai lint/schema issues were not treated as passing checks.

The new 60,000-voxel ROS regression verifies multiple packets per scheduling interval, exact shared and per-class credit accounting, individual packets at most 1200 bytes, and offered throughput above 75% of the 10 kB/s allocation. The first final run measured 9178 bytes/s. This is a desktop synthetic workload, not a Jetson/radio throughput measurement.

A 20-iteration desktop candidate-selection benchmark on 60,000 entries measured 13.25 ms mean for full ordering and 3.44 ms for bounded 2400-entry ordering. The regression verifies that the bounded selection matches the full priority order, including deterministic ties, and that an indexed ACK cannot resolve a superseding entry.

Receiver integration checks measured decoding, explicit unavailable end-to-end timing, old voxel stamps versus a recent packet scan header, and empty accepted sets. Radio tests cover sub-second poll jitter, read failures, cache expiry, inherited export age and AP association output. Analysis tests cover per-epoch cumulative totals, signed disturbance and unresolved one-way latency.

A local ROS service smoke test verified initial parameter snapshots, subsequent parameter events and artifact fingerprints in SQLite (`/tmp/surf-config-smoke-m88llb0m`). Python compilation and `git diff --check` passed. Logs are `/tmp/surf-indoor-final-tests.log`, `/tmp/surf-indoor-scheduler-tests.log`, and `/tmp/surf-indoor-bonxai-tests.log`.

## Follow-up after `LQR_indoor_test_2`

The second run achieved 9964 application bytes/s against the configured 10000,
but exposed 157 BACKLOG ACK timeouts for packets already observed at the receiver.
Both `/realtime_ack` and `/sync_ack` bridge inputs now use bounded FIFO queues,
with a real-message burst regression covering 32 consecutive IDs per class.

Routine debt traversal now uses an active pending index; retained resolved state
is scanned only by explicit recovery. Timeout/defer handling, scoring, age/state
metrics, and per-class candidate collection now share one active-set traversal,
reported as `debt_cycle_ms`. Selection ordering remains timed per class. A default
10 percent candidate reserve selects entries pending at least 30 seconds oldest
first, preventing the capped age score from starving the same early coordinates
forever. See the second run's [full review](../../../../experiments/LQR_indoor_test_2/review.md).

## Follow-up after `LQR_indoor_test_3`

The third run had no ACK timeouts, bounded oldest pending age near 30.9 seconds,
and restored the median controller interval to 100 ms. Phase telemetry showed
that separate active-set passes still consumed most callback time: advance,
scoring, metrics, and candidate collection repeatedly visited the same entries.

Those operations now run in one `InformationDebt::cycle` traversal. It performs
timeout/defer transitions, updates priorities and debt, accumulates all pending
telemetry, and partitions selectable entries by class. Only bounded candidate
ordering and encoding remain afterward. `debt_cycle_ms` measures the fused pass;
the three legacy per-pass timing fields are retained as zero for schema clarity.
See the third run's [full review](../../../../experiments/LQR_indoor_test_3/review.md).
