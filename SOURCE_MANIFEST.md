# Source and dependency boundary

This repository owns the `surf_*` packages at its root. A ROS workspace should
clone this repository below `src/`. Required mapping repositories are pinned
below `dependencies/` as Git submodules so one recursive clone obtains the
complete stack.

The repository's `.gitmodules` and gitlinks pin the external mapping
repositories used by the Jazzy system:

| Repository | Revision |
| --- | --- |
| Rolling Bonxai | `181e420a4d8cb5e784705e0e234b7c89ec11fd36` |
| Caltech Mapping | `93e92326a46d0f0dcd02406556ab7faf1985e52e` |
| GLIM | `9ad7444af3b2dc529df64bb6a391fc0341c57015` |
| GLIM ROS 2 | `4d4ec524ccf1b02aa09b0af2af767ecc54343798` |

`LIO-Localization` is deliberately outside this public repository. The private
parent workspace supplies it for the existing full Jazzy bringup; deployment
against another robot uses that robot's own localization and sensing stack.
