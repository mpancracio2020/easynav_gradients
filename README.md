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

The intended release license is Apache-2.0. See [LICENSE](LICENSE) for the license text and [the licensing review](docs/licensing.md) for the remaining provenance checks. Existing GPL notices in the planner sources are retained until the applicable rights and upstream revisions are confirmed. This private development snapshot is not yet cleared for public distribution.

## Acknowledgments

This planner was developed as a final degree project and integrates with [EasyNavigation](https://github.com/EasyNavigation/EasyNavigation) and [NavMap](https://github.com/EasyNavigation/NavMap). See also [easynav_plugins](https://github.com/EasyNavigation/easynav_plugins).
