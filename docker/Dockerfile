ARG ROS_DISTRO=jazzy
ARG ROS_SNAPSHOT=2026-09-11
ARG ZENOH_VERSION=1.8.0

FROM ros:jazzy@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8 AS base
ARG ROS_DISTRO
ARG ROS_SNAPSHOT
ARG ZENOH_VERSION

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE_DIR=/workspace

COPY docker/ros-snapshot-archive-keyring.asc /usr/share/keyrings/ros-snapshot-archive-keyring.asc
RUN rm /etc/apt/sources.list.d/ros2.sources \
      || { echo "image build failed: operation: replace the ROS apt source with snapshot ${ROS_SNAPSHOT}; cause: rm exited non-zero" >&2; exit 1; }; \
    printf 'Types: deb\nURIs: http://snapshots.ros.org/%s/%s/ubuntu\nSuites: noble\nComponents: main\nSigned-By: /usr/share/keyrings/ros-snapshot-archive-keyring.asc\n' \
      "${ROS_DISTRO}" "${ROS_SNAPSHOT}" > /etc/apt/sources.list.d/ros2-snapshot.sources

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ccache \
    python3-pip \
    python3-colcon-common-extensions \
    python3-scipy \
    "ros-${ROS_DISTRO}-ament-cmake-python" \
    "ros-${ROS_DISTRO}-ament-package" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-rmw-zenoh-cpp" \
    "ros-${ROS_DISTRO}-foxglove-bridge" \
    "ros-${ROS_DISTRO}-foxglove-msgs" \
    "ros-${ROS_DISTRO}-mqtt-client" \
    libgtest-dev \
    libnanoflann-dev \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-twist-mux" \
    "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    ca-certificates \
    curl \
    git \
    vim \
    nano \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-libav \
    "ros-${ROS_DISTRO}-camera-info-manager" \
    "ros-${ROS_DISTRO}-cv-bridge" && \
    rm -rf /var/lib/apt/lists/*

ARG ZENOH_REPO=https://download.eclipse.org/zenoh/debian-repo
RUN staging=/tmp/zenoh; \
    curl -fsSLo /tmp/zenohd.deb ${ZENOH_REPO}/${ZENOH_VERSION}/zenohd_${ZENOH_VERSION}_amd64.deb \
      || { echo "image build failed: operation: download zenohd ${ZENOH_VERSION} from ${ZENOH_REPO}; cause: curl exited non-zero" >&2; exit 1; }; \
    dpkg-deb -x /tmp/zenohd.deb "$staging" \
      || { echo "image build failed: operation: extract zenohd ${ZENOH_VERSION} into $staging; cause: dpkg-deb exited non-zero" >&2; exit 1; }; \
    rm -f /tmp/zenohd.deb; \
    install -m 0755 "$staging/usr/bin/zenohd" /usr/bin/zenohd \
      || { echo "image build failed: operation: install zenohd ${ZENOH_VERSION} into /usr/bin; cause: install exited non-zero" >&2; exit 1; }; \
    rm -rf "$staging"; \
    zenohd --version | grep -q "^zenohd v${ZENOH_VERSION} " \
      || { echo "image build failed: operation: verify the installed zenohd version; cause: zenohd --version did not report ${ZENOH_VERSION}" >&2; exit 1; }; \
    zenohd --version | head -1

COPY docker/pip-constraints.txt /etc/pip-constraints.txt
RUN pip install --break-system-packages --no-cache-dir -c /etc/pip-constraints.txt pyproj==3.8.0 pydantic==2.13.5

COPY docker/ros-workspace-exec /usr/local/bin/ros-workspace-exec
RUN chmod 0755 /usr/local/bin/ros-workspace-exec

WORKDIR ${WORKSPACE_DIR}

RUN mkdir -p .ccache && chmod 1777 .ccache

RUN groupadd -f input

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN cat <<'EOF' >> /etc/bash.bashrc
if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -n "${WORKSPACE_DIR}" ] && [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
EOF

ENTRYPOINT ["/usr/local/bin/ros-workspace-exec"]
CMD ["bash"]

FROM base AS dev

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-joy" \
    "ros-${ROS_DISTRO}-teleop-twist-joy" && \
    rm -rf /var/lib/apt/lists/*

FROM base AS simulation

# Nav2 MPPI CPU compatibility fix for CPUs without AVX2/FMA
COPY docker/nav2_cpu_fix/libmppi_controller.so /opt/ros/${ROS_DISTRO}/lib/libmppi_controller.so
COPY docker/nav2_cpu_fix/libmppi_critics.so /opt/ros/${ROS_DISTRO}/lib/libmppi_critics.so

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-ros-gz" && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
    file \
    gdb \
    libssl-dev \
    libxml2-dev \
    libxml2-utils \
    ninja-build \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    rsync \
    zip && \
    rm -rf /var/lib/apt/lists/*

COPY src/third_party/PX4-Autopilot/Tools/setup/requirements.txt /tmp/px4_requirements.txt
RUN pip install --break-system-packages --no-cache-dir \
    -c /etc/pip-constraints.txt \
    -r /tmp/px4_requirements.txt && \
    rm -f /tmp/px4_requirements.txt

RUN git config --system --add safe.directory '*' && \
    ln -s /workspace/src/third_party/PX4-Autopilot /opt/px4

FROM base AS physical

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
    can-utils \
    chrony \
    iproute2 \
    python3-can \
    "ros-${ROS_DISTRO}-joy" \
    "ros-${ROS_DISTRO}-teleop-twist-joy" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-diff-drive-controller" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-microstrain-inertial-driver" \
    libapr1-dev \
    libaprutil1-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    zlib1g-dev \
    libeigen3-dev \
    nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

FROM physical AS physical-runtime

# hadolint ignore=DL3022
COPY --from=install . ${WORKSPACE_DIR}/install
