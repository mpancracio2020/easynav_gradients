# Installation (ROS 2 Jazzy)

## Prerequisites

Ubuntu 24.04, ROS 2 Jazzy, a C++23 compiler, CMake 3.20 or newer, Git, colcon, rosdep and vcstool. Start from a terminal that has not sourced an unrelated ROS overlay.

The preferred isolated route is [Docker](../docker/README.md). The root easynav_gradients.repos pins the four source revisions resolved in the successful local Docker build. The base image and apt dependencies are not fully locked.

## Build in a separate overlay

If you already have a working EasyNav/NavMap workspace at ~/workshop_ws, build this package in a separate overlay to avoid altering the original development checkout:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workshop_ws/install/setup.bash
mkdir -p ~/gradients_ws/src
cd ~/gradients_ws/src
git clone -b develop https://github.com/mpancracio2020/easynav_gradients.git
cd ~/gradients_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select easynav_gradients --symlink-install
source install/setup.bash
ros2 pkg prefix easynav_gradients
```

The repository is private during review; clone access is required. If colcon warns that easynav_gradients already exists in the underlay, rebuild an underlay without the original planner when possible. Only use --allow-overriding easynav_gradients when you have checked overlay compatibility. Do not place both copies in the same source workspace. The reported package prefix should be in the new overlay.

## Optional: capture dependency revisions from an existing workspace

In the original working workspace, install vcstool if needed, then run:

```bash
cd ~/workshop_ws
vcs export --exact src > /tmp/easynav-workspace-exact.repos
vcs status src
colcon list
```

Review the exported file before sharing: remove unrelated repositories, the private planner and any private URLs or credentials. Retain only required dependencies, with the full commit SHA for each. Check whether those commits are publicly accessible. A dirty dependency checkout is not captured by a commit SHA alone; inspect local modifications and record any required patches separately.

This is optional and unnecessary for the Docker route, whose source revisions are already pinned. Do not replace the tested manifest with unvalidated latest revisions.

## Validate installation

Check the shared library and plugin description:

```bash
prefix=$(ros2 pkg prefix easynav_gradients)
test -f "$prefix/lib/libeasynav_gradients.so"
test -f "$prefix/share/easynav_gradients/easynav_gradients_plugins.xml"
test -f "$prefix/share/easynav_gradients/config/params.yaml"
```

These are installation checks, not proof that pluginlib can instantiate the class or that the planner produces a valid path. Follow [usage.md](usage.md) for simulation validation. The maintainer confirmed the isolated Docker build and installed library; plugin instantiation and functional simulation remain pending.
