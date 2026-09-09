# SURF patch provenance

Vendored from https://github.com/brow1633/network_bridge, tag 3.0.0,
commit `b58c0fe19cbb52d1b3ba8fe3b6873f82124d767c` (MIT).

Local changes:

- Opt-in per-topic FIFO message/byte limits, with explicit overflow logging.
- FIFO subscription DDS depth matches its configured message limit.
- Receiving `CompressedVoxelDelta` publishers retain 256 DDS samples.
- Queue regressions and a ROS/UDP burst integration test.
- Legacy UDP/TCP tests wait for DDS discovery instead of assuming a 150 ms delay;
  launch-test timeouts allow discovery and cleanup to finish.

Latest-value forwarding remains the default for other topics. Build this
workspace package on both robots; the stock binary package lacks these fixes.
See `../../docs/transport_and_state_fixes.md` for operation and limitations.
