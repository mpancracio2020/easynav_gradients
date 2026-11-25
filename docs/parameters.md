# Planner parameters

Parameters are read at initialization under the configured plugin instance name (example: gradients). The values below are from config/params.yaml, not necessarily the C++ fallback defaults.

| Parameter | Example | C++ default | Valid range / behavior |
| --- | ---: | ---: | --- |
| continuous_replan | false | true | Boolean; regenerate path each update if true |
| max_iterations | 500 | 500 | Positive integer; invalid values reset to 500 |
| goal_change_epsilon | 0.001 | 0.001 | Positive finite value, meters; componentwise goal position comparison |
| smoothing_factor | 0.4 | 0.4 | [0, 1]; finite values clamped, nonfinite reset to 0.4 |
| potential_drop_weight | 0.75 | 0.75 | Finite, nonnegative; invalid resets to 0.75 |
| robot_width | 0.8 | 0.0 | Meters; zero disables edge-width filter; negative/nonfinite resets to zero |
| elastic_w_data | 0.10 | 0.10 | Finite, nonnegative; invalid resets to 0.10 |
| elastic_w_smooth | 0.35 | 0.35 | Finite, nonnegative; invalid resets to 0.35 |
| elastic_max_iterations | 15 | 15 | Positive integer; invalid resets to 15 |
| elastic_convergence_threshold | 0.002 | 0.002 | Positive finite value, meters; invalid resets to 0.002 |

## Tuning

Start with the supplied configuration. Change one parameter at a time and evaluate path success, length, clearance and latency on identical maps and goals. A larger potential_drop_weight favors larger potential decreases relative to directional alignment. A larger smoothing_factor blends more of the neighborhood average into the original gradient. Elastic Band weights control attraction toward original waypoints and local smoothing; greater values do not necessarily guarantee better or valid paths.

The width restriction is not a footprint collision guarantee. Invalid width values disable the filter in this baseline; verify the configuration before use.

The baseline does not expose profiling switches: both CSV outputs are written automatically. The optional profiling refactor is separate and not part of this documented version.
