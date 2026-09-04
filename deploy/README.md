# Deployment assets

The `local`, `jetson`, `network`, `halow-telemetry`, and `clock-sync`
directories install the existing host-level radio, clock, and telemetry
support. `jetson/` retains the full current Jazzy/Livox deployment, while
`docker/` is the robot-stack-independent container entry point.

Use `docker/Dockerfile.jazzy` and `docker/compose.yaml` for an isolated build of
this extracted stack on the current development distribution. They deliberately
exclude Livox, FAST-LIO, GLIM, ScanLock, RViz, and PX4. The container joins the
host network so it can communicate with an existing ROS graph.

The Docker assets have not been converted to Humble. Make that conversion on
the future Humble branch together with source compatibility testing.

For Compose, paths inside `robot.yaml` must use the container mount points:
`transport_config: /config/transport.yaml`, a map below `/data/maps`, and a
telemetry directory below `/data/telemetry`. The host paths for those mounts
belong in `docker/.env`.

Host installers remain hardware-specific and must be run from this extracted
directory because they install companion files by relative path.
