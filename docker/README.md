# Isolated ROS 2 Jazzy build

This build uses the official ROS Jazzy base image and public upstream Git commits pinned in the root .repos file. The maintainer successfully compiled and installed the planner using these resolved revisions on 2026-11-25. A fresh CI build with the pinned manifest and functional simulation remain pending. No host ROS workspace, credentials or device access is required. The image is not published to a registry.

From the repository root:

```bash
docker build --progress=plain -t easynav-gradients:jazzy -f docker/Dockerfile .
docker run --rm easynav-gradients:jazzy ros2 pkg prefix easynav_gradients
docker run --rm -it easynav-gradients:jazzy
```

For machines with limited memory use --build-arg BUILD_JOBS=1. The build installs dependencies for the imported repositories, which can require substantial disk space. It builds the planner and its dependency closure with upstream tests disabled, then rebuilds only the planner with testing enabled and runs eight tests covering plugin discovery, instantiation, parameter initialization and synthetic mesh planning, including disconnected goals, path reuse, topology changes and final orientation. The maintainer reported 8/8 passing in 153 ms. These tests do not establish full collision clearance or simulation integration.

Inspect the exact resolved revisions after a successful build:

```bash
docker run --rm easynav-gradients:jazzy cat /opt/dependencies-resolved.repos
```

Run the complete automated test suite again from the built image:

```bash
docker run --rm easynav-gradients:jazzy /opt/gradients_ws/build/easynav_gradients/tests/test_plugin_loading
```

The CI workflow runs the same Dockerfile and uploads the resolved revision list after a successful build. The four source commits are now pinned. The base-image digest, apt package versions and any additional transitive inputs are not fully locked, so this does not guarantee bit-for-bit reproducibility.

If compilation fails, inspect the first error in the Docker build log. Do not modify the working thesis workspace. The original planner remains the reference and API compatibility changes require separate review.

Official dependency guide: https://easynavigation.github.io/build_install/
