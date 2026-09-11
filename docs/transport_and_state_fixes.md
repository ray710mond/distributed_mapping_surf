# Transport and state maintenance

Map packets use the bounded FIFO support in the vendored `network_bridge`.
Both DELTA (`/realtime_tx`) and BACKLOG (`/sync_tx`) use the same HaLow UDP
bridge, with 256-packet / 307200-byte queue limits per topic. Overflow retains
outstanding sender debt for reselection after ACK timeout. Obsolete queued
observations remain harmless because the receiver validates each voxel stamp.

Sender `SpatialMap` block indexing and `DynamicExpiry` scheduling remain in use
for local mask queries and dynamic evidence expiry. These avoid map-wide work
for spatial filtering. Debt scoring and priority sorting still scale with the
outstanding map; benchmark those costs on the Jetson before increasing scan rate.

The prior periodic snapshot protocol and adaptive policy are retired. See
[information allocation](information_allocation.md) for current recovery,
byte budgeting, temporal consistency and tests.
