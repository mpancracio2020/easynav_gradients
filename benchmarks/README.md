# Benchmark scripts

Original analysis scripts retained from the thesis working copy. Raw experimental data and plots are intentionally excluded.

Python dependencies: pandas, matplotlib.

```bash
python3 benchmarks/scripts/plot_profile.py /tmp/gradient_planner_profile.csv benchmarks/results/profile
python3 benchmarks/scripts/plots.py
```

The second script reads `/tmp/planner_metrics.csv`. These scripts have not been execution-tested against fresh data.
