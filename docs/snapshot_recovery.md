# Latest full snapshots and recovery

A newer full snapshot supersedes an older full snapshot as a whole. A newer
realtime delta does not supersede a full snapshot: it carries only selected
changes and cannot repair all missing map state.

## Sender and bridge

Pending acknowledgments no longer block periodic full-snapshot generation.
Generating a replacement cancels unsent old chunks and resets the retry state.
Only an ACK matching the current source, epoch and snapshot version can clear
the pending snapshot. Requests are rate limited by the base sync interval (at
least one second), while periodic generation retains the adaptive sync interval.

The `/sync_tx.latest_snapshot: true` bridge option discards queued chunks from
older snapshot versions when a new generation arrives, retaining *every chunk*
of the chosen snapshot. Its version watermark survives an empty queue, so a
late older retry cannot restore obsolete work. The option requires a bounded
FIFO and a single `CompressedVoxelDelta` source on the topic. A new map epoch
restarts version numbering. Bytes already handed to TCP cannot be withdrawn;
the receiver rejects superseded versions too.

The vendored bridge now explicitly depends on `surf_multirobot_msgs` for this
opt-in snapshot metadata handling. Ordinary FIFO and latest-value topics retain
their existing behavior.

## Receiver and atomic application

The highest valid full-snapshot generation seen determines which partial
assembly is retained. Late chunks from older full snapshots are rejected and
reported in delivery telemetry. Decoding must succeed before a newer snapshot
can supersede the active assembly. Duplicate chunks from the selected snapshot
remain valid, and a duplicate of an already applied snapshot reissues its ACK
without resetting or republishing the map.

All completed, applied realtime updates are retained after the last full
snapshot, including updates applied before the first new snapshot chunk arrives.
Upon completion, the receiver appends the retained newer operations to the
snapshot in version order and publishes one full replacement with the newest
applied version. The map consumer applies records in order, so newer occupied,
free and deletion records win. This avoids both a rollback window and the
consumer rejecting an older snapshot before replay. The ACK still names the
original wire snapshot version; newer retained operations remain available for
the next snapshot.

`maximum_realtime_history_bytes` defaults to 64 MiB per source and bounds
accounted retained message storage. If history needed by an old snapshot has
been evicted, the receiver rejects that snapshot explicitly and requests a
newer one. It also rejects incompatible resolution/frame history rather than
publishing a replacement that would lose newer state. Memory overhead for
container allocation is additional to the accounting limit.

## Validation and rollout

Local validation: all four affected C++ packages built successfully. The final
colcon report has 76 test entries, zero errors/failures, and 14 skipped cppcheck
entries. All nine new behavioral regressions passed, along with the existing
transport, codec, adaptive-controller, and state tests.

Regression coverage includes sender generation with no ACKs, late obsolete
ACKs, a disconnected bridge backlog replaced by a newer generation, a snapshot
delayed while realtime advances, interleaved old/new partial snapshots,
duplicate ACK recovery, and bounded-history exhaustion. Receiver tests exercise
the production ROS subscriptions and codec, modeling a disconnect by withholding
snapshot delivery while realtime continues. They do not emulate every TCP or
radio failure mode.

Rebuild and source `surf_multirobot_comms`, `surf_drone`, `surf_humanoid`,
`network_bridge`, and `surf_bringup` on the relevant hosts, or rebuild the full
workspace/container. A hardware disconnect/reconnect run remains necessary to
validate radio/TCP recovery end to end.
