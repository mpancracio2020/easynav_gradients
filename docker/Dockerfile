FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=2
ENV ROS_DISTRO=jazzy
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions \
    python3-vcstool python3-rosdep python3-yaml libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/gradients_ws
COPY easynav_gradients.repos /tmp/dependencies.repos
RUN mkdir -p src && vcs import --recursive src < /tmp/dependencies.repos \
    && vcs export --exact src > /opt/dependencies-resolved.repos
COPY . src/easynav_gradients
RUN source /opt/ros/jazzy/setup.bash \
    && python3 src/easynav_gradients/scripts/check_metadata.py \
    && colcon list \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-up-to easynav_gradients \
      --executor sequential --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN source install/setup.bash \
    && prefix="$(ros2 pkg prefix easynav_gradients)" \
    && test -f "$prefix/lib/libeasynav_gradients.so" \
    && test -f "$prefix/share/easynav_gradients/easynav_gradients_plugins.xml" \
    && test -f "$prefix/share/easynav_gradients/config/params.yaml"
COPY docker/entrypoint.sh /usr/local/bin/gradients-entrypoint
RUN chmod +x /usr/local/bin/gradients-entrypoint
ENTRYPOINT ["/usr/local/bin/gradients-entrypoint"]
CMD ["bash"]
