# Compact measured free rays

The occupied-origin receiver extrapolation profile sets
`mapping.new_free_ray_stride=0`. Each newly transmitted occupied hit carries
its own scan origin (SVD3 flag 2); the receiver traces the free prefix to that
hit using the voxel center as endpoint. No novel FREE voxel samples are sent
in this profile. Explicit clears of known obstacles remain FREE records, and
expiry remains UNKNOWN. Receiver reconstruction is bounded by the configured
ray work, range, height, and radius limits. It fills measured lines to delivered
hits; adjacent hit rays may provide dense coverage, but this does not certify
clearance between them or guarantee a connected planning region. Existing
occupied records from before this protocol do not have scan origins and are
not retroactively expanded.

The drone keeps per-voxel occupied updates, explicit clears of known occupied
cells, and expiry withdrawals. Only newly observed free space is sampled (one
safe beam prefix in 256 eligible rays in the hardware profile). Each sampled FREE record retains the
sensor origin and the measured beam endpoint from its own scan. The SVD3 codec
keeps that geometry with the record across DELTA, BACKLOG, ACK retries, and
receiver recovery. A packet's common `sensor_origin` is not used for ray replay.

The sender records ray geometry only for prefixes before any current-scan
endpoint, humanoid mask, or occupied cell that has not been cleared. The receiver
walks the same voxel ray to the sampled coordinate and applies range, height,
self, and communication-radius filters. It never fills gaps between unrelated
samples. Exact obstacle clears remain ordinary FREE voxel records so the sender's
miss threshold remains authoritative. UNKNOWN expiry records still withdraw
transient occupancy without claiming free space.

The hardware profile disables sender-side BACKLOG connected-component grouping.
It operated on the sparse sampled records and could spend a packet on a free
component after its seed. BACKLOG now uses per-record priority; DELTA already did.
Each delivered ray reconstructs contiguous measured free evidence locally.

SVD3 changes the `VoxelDelta` interface and compressed payload. Drone, humanoid,
message, and codec packages must be deployed together. The free-ray filter and
resolution settings on the receiver must match the drone settings. The packet
size credit and ACK/recovery logic continue to account for the transmitted
sampled records; receiver-expanded voxels are local map updates.
Occupied-only and ordinary clear packets still use SVD2, avoiding ray geometry
overhead on the urgent obstacle traffic.
