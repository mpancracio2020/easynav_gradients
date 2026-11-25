# EasyNav Gradients

Gradient-based global path planner for EasyNav using NavMap, developed for ROS 2 Jazzy.

**Development baseline — private, not ready for public distribution.**

The planner implements potential propagation, gradient-based path generation, caching and Elastic Band smoothing.

## Build

Place this package in a ROS 2 Jazzy workspace containing compatible EasyNav and NavMap dependencies. Exact dependency revisions and clean-build instructions remain to be verified. Do not include two copies of the `easynav_gradients` package in the same workspace.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to easynav_gradients
source install/setup.bash
```

Plugin identifier: `easynav_navmap_gradients/GradientPlanner`. Example parameters: `config/params.yaml`.

## Benchmarks

See [benchmarks/README.md](benchmarks/README.md). Profiling remains enabled as in the original baseline.

## License status

Publication is blocked pending verification of the original licensing and third-party attribution. The supplied LICENSE and package manifest declare Apache-2.0, while the planner source headers and original README declare GPL-3.0-or-later / GPL v3 respectively. Original declarations are preserved; this baseline does not resolve or replace them.
