# Benchmark scripts

Original analysis scripts retained from the thesis working copy. Raw experimental data and plots are intentionally excluded.

Python dependencies: pandas, matplotlib.

```bash
python3 benchmarks/scripts/plot_profile.py /tmp/gradient_planner_profile.csv benchmarks/results/profile
python3 benchmarks/scripts/plots.py
```

The second script reads `/tmp/planner_metrics.csv`. These scripts have not been execution-tested against fresh data.

## Enabling diagnostics

Both CSV outputs are disabled by default. Enable the corresponding parameters
under the configured planner plugin namespace before initialization:

```yaml
gradients:
  profiling_enabled: true
  metrics_enabled: true
  profiling_output_file: "/tmp/gradient_planner_profile.csv"
  metrics_output_file: "/tmp/planner_metrics.csv"
```

Use a separate pair of output files for every concurrent planner instance.
Parent directories must already exist. Files are appended, so use fresh paths
for independent benchmark runs. Profiling records wall-clock timestamps and
monotonic elapsed durations; metrics retain the original three-column format.
Timing measurements with diagnostics enabled include instrumentation overhead.
