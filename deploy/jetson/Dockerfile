# syntax=docker/dockerfile:1
FROM ros:jazzy-perception-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG LIVOX_ROS_DRIVER2_REF=1.2.6
ARG PX4_MSGS_REF=release/1.15
ARG BUILD_JOBS=4

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV MAKEFLAGS=-j${BUILD_JOBS}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gpg \
        iproute2 \
        iw \
        libapr1-dev \
        libssl-dev \
        libzstd-dev \
        pkg-config \
        python3-colcon-common-extensions \
        python3-dev \
        python3-rosdep \
        ros-jazzy-ament-cmake-auto \
        ros-jazzy-network-bridge \
        ros-jazzy-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# GLIM is built from the pinned workspace submodules. Its ARM64 CPU
# dependencies are provided by the GLIM project's Noble PPA; CUDA remains off
# in this portable baseline and can be enabled later with a JetPack-matched
# gtsam_points build.
RUN curl -fsSL \
        https://koide3.github.io/ppa/ubuntu2404/KEY.gpg \
        -o /tmp/koide3-ppa.gpg \
    && gpg --dearmor \
        --output /etc/apt/trusted.gpg.d/koide3-ppa.gpg \
        /tmp/koide3-ppa.gpg \
    && echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3-ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" \
        > /etc/apt/sources.list.d/koide3-ppa.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-all-dev \
        libfmt-dev \
        libgtsam-points-dev \
        libiridescence-dev \
        libmetis-dev \
        libomp-dev \
        libspdlog-dev \
    && dpkg-query -W libiridescence-dev \
    && rm -f /tmp/koide3-ppa.gpg \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2 is supplied as the named "livox_sdk2" build context. It is still
# compiled inside Noble so the library ABI matches the container rather than
# the Jetson host OS.
WORKDIR /opt/vendor
COPY --from=livox_sdk2 . Livox-SDK2/
RUN if ! grep -Rqs "kLivoxLidarTypeMid360s" Livox-SDK2/include; then \
        echo "Livox-SDK2 is too old for livox_ros_driver2 ${LIVOX_ROS_DRIVER2_REF}; use SDK2 v1.3.1 or newer" >&2; \
        exit 1; \
    fi \
    && rm -rf Livox-SDK2/build \
    && cmake -S Livox-SDK2 -B Livox-SDK2/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-include cstdint" \
    && cmake --build Livox-SDK2/build --parallel "${BUILD_JOBS}" \
    && cmake --install Livox-SDK2/build \
    && ldconfig \
    && rm -rf Livox-SDK2/.git Livox-SDK2/build

RUN mkdir -p /opt/livox_ws/src \
    && git clone --branch "${LIVOX_ROS_DRIVER2_REF}" --depth 1 \
        https://github.com/Livox-SDK/livox_ros_driver2.git \
        /opt/livox_ws/src/livox_ros_driver2 \
    && cp -f \
        /opt/livox_ws/src/livox_ros_driver2/package_ROS2.xml \
        /opt/livox_ws/src/livox_ros_driver2/package.xml \
    && cp -rf \
        /opt/livox_ws/src/livox_ros_driver2/launch_ROS2 \
        /opt/livox_ws/src/livox_ros_driver2/launch \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/livox_ws \
    && colcon build \
        --cmake-args \
            -DROS_EDITION=ROS2 \
            -DDISTRO_ROS=jazzy \
            -DCMAKE_BUILD_TYPE=Release \
    && test -f \
        /opt/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash \
    && source /opt/livox_ws/install/setup.bash \
    && ros2 pkg prefix livox_ros_driver2 \
    && rm -rf /opt/livox_ws/build /opt/livox_ws/log \
        /opt/livox_ws/src/livox_ros_driver2/.git

# The uXRCE-DDS Agent stays on the Jetson host. Only the ROS interface package
# is built here, against Jazzy, so the container can deserialize the DDS topics
# without depending on the host's Humble install space. PX4_MSGS_REF must match
# the PX4 firmware; pinning the commit used by the host workspace is safest.
RUN mkdir -p /opt/px4_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git \
        /opt/px4_ws/src/px4_msgs \
    && git -C /opt/px4_ws/src/px4_msgs checkout "${PX4_MSGS_REF}" \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/px4_ws \
    && colcon build \
        --parallel-workers "${BUILD_JOBS}" \
        --merge-install \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
    && source /opt/px4_ws/install/local_setup.bash \
    && ros2 interface show px4_msgs/msg/SensorCombined >/dev/null \
    && rm -rf /opt/px4_ws/build /opt/px4_ws/log \
        /opt/px4_ws/src/px4_msgs/.git

WORKDIR /opt/surf_ws
COPY surf_ws/src/distributed_mapping_surf src/distributed_mapping_surf
COPY surf_ws/src/LIO-Localization src/LIO-Localization

# relocalization_bringup consumes GLIM without declaring it as a package
# dependency. Install the exact pinned source as a CMake underlay so the
# full-workspace colcon build can configure deterministically.
RUN if [[ ! -f src/distributed_mapping_surf/dependencies/caltech_mapping/caltech_bonxai_builder/package.xml || \
          ! -f src/distributed_mapping_surf/dependencies/caltech_mapping/caltech_map_bringup/package.xml || \
          ! -f src/distributed_mapping_surf/dependencies/glim/package.xml || \
          ! -f src/distributed_mapping_surf/dependencies/glim_ros2/package.xml || \
          ! -f src/LIO-Localization/ws_lio_loc/src/relocalization_bringup/package.xml ]]; then \
        echo "A required mapping submodule is missing; run git submodule update --init --recursive before building" >&2; \
        exit 1; \
    fi \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --skip-keys "livox_ros_driver2" \
        --rosdistro jazzy \
        -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/jazzy/setup.bash \
    && source /opt/livox_ws/install/setup.bash \
    && source /opt/px4_ws/install/local_setup.bash \
    && cmake \
        -S src/distributed_mapping_surf/dependencies/glim \
        -B /tmp/glim-underlay-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/glim-underlay \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_WITH_VIEWER=ON \
        -DBUILD_WITH_OPENCV=OFF \
    && cmake --build /tmp/glim-underlay-build --parallel "${BUILD_JOBS}" \
    && cmake --install /tmp/glim-underlay-build \
    && rm -rf /tmp/glim-underlay-build \
    && export CMAKE_PREFIX_PATH="/opt/glim-underlay:${CMAKE_PREFIX_PATH:-}" \
    && colcon build \
        --parallel-workers "${BUILD_JOBS}" \
        --merge-install \
        --install-base /opt/surf \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DBUILD_WITH_CUDA=OFF \
            -DBUILD_WITH_VIEWER=ON \
            -DBUILD_WITH_OPENCV=OFF \
            -DBUILD_WITH_CV_BRIDGE=OFF \
    && source /opt/ros/jazzy/setup.bash \
    && source /opt/livox_ws/install/setup.bash \
    && source /opt/px4_ws/install/local_setup.bash \
    && source /opt/surf/local_setup.bash \
    && ros2 pkg prefix livox_ros_driver2 \
    && ros2 pkg prefix caltech_bonxai_builder \
    && ros2 pkg prefix caltech_map_bringup \
    && ros2 pkg prefix glim \
    && ros2 pkg prefix glim_ros \
    && ros2 pkg prefix relocalization_bringup \
    && ros2 pkg prefix surf_slam \
    && test -x /opt/surf/lib/relocalization_bringup/glim_optimize_export \
    && test -x /opt/surf/lib/caltech_bonxai_builder/bonxai_pcd_converter \
    && test -x /opt/surf/lib/caltech_bonxai_builder/pcd_to_ros_occupancy.py \
    && rm -rf build log

COPY surf_ws/src/distributed_mapping_surf/deploy/jetson/ros_entrypoint.sh /ros_entrypoint.sh
COPY surf_ws/src/distributed_mapping_surf/deploy/jetson/render_mid360_config.py /usr/local/bin/render_mid360_config.py
COPY surf_ws/src/distributed_mapping_surf/deploy/jetson/root.bashrc /root/.bashrc
RUN chmod 0755 /ros_entrypoint.sh /usr/local/bin/render_mid360_config.py \
    && mkdir -p /data/telemetry /data/ros-logs /opt/surf_ws/maps

ENV ROS_DISTRO=jazzy
ENV ROS_LOG_DIR=/data/ros-logs

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["sleep", "infinity"]
