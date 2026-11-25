# Usage

EasyNav Gradients is a PlannerMethodBase plugin, not a standalone ROS node. Run it with an existing EasyNav bringup and a NavMap provider.

## Configure an existing bringup

1. Locate the planner plugin configuration in a working EasyNav bringup.
2. Replace the existing A* planner class identifier with easynav_navmap_gradients/GradientPlanner, preserving the upstream configuration schema.
3. Use the instance name gradients (or rename the parameter prefix consistently).
4. Merge the planner parameters from config/params.yaml into the bringup's parameter file.
5. Launch the existing simulation using its documented command.

The exact upstream plugin-list parameter key and launch file depend on the EasyNav revision and are not yet verified. The YAML in this repository is a parameter fragment, not a complete bringup configuration.

## Runtime contract

Inputs are read from NavState, not directly from standalone subscriptions:

| Key | Type |
| --- | --- |
| goals | nav_msgs/msg/Goals |
| robot_pose | nav_msgs/msg/Odometry |
| map.navmap | navmap::NavMap |

The goal header must use the map frame. The planner selects the first goal. The output is stored under the path key as nav_msgs/msg/Path. A ROS path is also published to the planner node's fully qualified name followed by /<plugin-instance>/path when subscribers exist. The potential layer is published on /navmap_layers when subscribers exist and the field is computed.

## Simulation checks

```bash
ros2 node list
ros2 topic list -t
ros2 param list /planner_node
ros2 param get /planner_node gradients.continuous_replan
ros2 topic info /planner_node/gradients/path
ros2 topic echo /planner_node/gradients/path --once
```

Adapt node and topic names to the actual namespace. Send goals through the existing EasyNav interface; the example simple_test node does not replace the required NavState integration.

Test a reachable goal, an unreachable goal, a changed goal orientation and a changed map. Inspect the path in RViz, verify its endpoint and ensure connecting segments remain navigable. Treat cancellation and invalid-input cases as known limitations until separately validated.

## Diagnostics

The original baseline always writes /tmp/gradient_planner_profile.csv and /tmp/planner_metrics.csv. Archive previous logs before starting a fresh run. Use a single planner instance when collecting measurements with these fixed output paths. See [benchmark instructions](../benchmarks/README.md).
