# Deployment assets

The `local`, `jetson`, `network`, `halow-telemetry`, and `clock-sync`
directories install the existing host-level radio, clock, and telemetry
support. `jetson/` retains the full current Jazzy/Livox deployment, while
`docker/` is the robot-stack-independent container entry point.

Use `docker/Dockerfile.jazzy` and `docker/compose.yaml` for an isolated build of
this extracted stack on the current development distribution. They deliberately
exclude Livox, FAST-LIO, GLIM, ScanLock, and PX4. The container joins the host
network so it can communicate with an existing ROS graph.

The Docker assets have not been converted to Humble. Make that conversion on
the future Humble branch together with source compatibility testing.

For Compose, paths inside `robot.yaml` must use the container mount points:
`transport_config: /config/transport.yaml`, a map below `/data/maps`, and a
telemetry directory below `/data/telemetry`. The host paths for those mounts
belong in `docker/.env`.

To inspect the local map before transport sends it, start the optional RViz
profile from `deploy/docker`:

```bash
xhost +local:docker
docker compose --profile rviz up --build
```

The RViz layout opens the portable drone topics, including
`/drone/bonxai/occupied_voxels`, `/drone/transport/drone_odometry`, and
`/drone/transport/humanoid_odometry`. Set `DISPLAY` in `docker/.env` if your
desktop session does not use `:0`; set `XAUTHORITY` only if your X11 session
requires an explicit authority file.

Host installers remain hardware-specific. Run `deploy/install-surf-cli.sh`
once from the checkout; the installed commands remember the repository path
and may then be invoked from any working directory.
