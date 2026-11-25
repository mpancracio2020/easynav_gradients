# Release validation

## Build and installation
- [x] Capture the four source revisions from the successful local Docker build and pin them in the manifest.
- [x] Compile in an isolated ROS 2 Jazzy Docker workspace (maintainer-reported success, 2026-11-25).
- [ ] Repeat in CI using the pinned manifest.
- [x] Verify installed library, plugin XML and configuration via Docker build checks; maintainer additionally confirmed the installed package and library.
- [x] Pass automated pluginlib discovery and instantiation smoke tests (maintainer-reported Docker run: 2/2 passed, 18 ms total).
- [ ] Initialize the plugin through the actual EasyNav bringup.

## Functional regression
- [ ] Compare the release candidate with the original thesis baseline using identical maps, goals and parameters.
- [x] Check reachable and unreachable goals, goal orientation, map changes and path reuse on synthetic triangular meshes (maintainer-reported Docker build: 8/8 tests passed, 153 ms total, including plugin loading and parameter initialization).
- [ ] Check path segments and robot footprint clearance independently of waypoint projection.
- [ ] Document cancellation and invalid-input behavior.

## Distribution
- [x] Verified upstream reference copyright and Apache-2.0 notices; maintainer confirmed distribution rights on 2026-11-25.
- [x] Basic tracked-file and history audit completed by maintainer; no matches for the tested secret patterns. This is not an exhaustive secret scan.
- [ ] Validate software citation metadata.
- [ ] Publish only after owner approval.

A successful static metadata check is not a substitute for compilation or simulation.
