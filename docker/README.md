# Isolated ROS 2 Jazzy build

This build uses the official ROS Jazzy base image and public EasyNav release tags listed in the root .repos file. They are candidates, not a certified compatible set. No host ROS workspace, credentials or device access is required. The image is not published to a registry.

From the repository root:

```bash
docker build --progress=plain -t easynav-gradients:jazzy -f docker/Dockerfile .
docker run --rm easynav-gradients:jazzy ros2 pkg prefix easynav_gradients
docker run --rm -it easynav-gradients:jazzy
```

For machines with limited memory use --build-arg BUILD_JOBS=1. The build installs dependencies for the imported repositories, which can require substantial disk space. It builds only the planner and its dependency closure; tests are disabled during compilation. The package installation checks are not a functional simulation or plugin-instantiation test.

Inspect the exact resolved revisions after a successful build:

```bash
docker run --rm easynav-gradients:jazzy cat /opt/dependencies-resolved.repos
```

The CI workflow runs the same Dockerfile and uploads the resolved revision list after a successful build. Pin the validated commit SHAs and base-image digest in a later release step; release tags and apt packages alone do not provide bit-for-bit reproducibility.

If compilation fails, inspect the first error in the Docker build log. Do not modify the working thesis workspace. The original planner remains the reference and API compatibility changes require separate review.

Official dependency guide: https://easynavigation.github.io/build_install/
