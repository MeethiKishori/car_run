# Controller Comparison Report

## Inputs

- PID runs: 2
- BC runs: 2

## Accuracy

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| rmse_cte | 0.3357 +- 0.0537 | 0.3073 +- 0.1141 |
| mae_cte | 0.2391 +- 0.0415 | 0.2416 +- 0.0943 |
| rmse_heading | 0.6649 +- 0.2116 | 0.3799 +- 0.0239 |
| mae_heading | 0.4328 +- 0.1287 | 0.2820 +- 0.0409 |
| final_goal_distance | 0.0602 +- 0.0019 | 0.6755 +- 0.3775 |

## Robustness

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| p95_abs_cte | 0.6585 +- 0.0455 | 0.6238 +- 0.1945 |
| max_abs_cte | 0.8294 +- 0.0067 | 0.7676 +- 0.2566 |
| high_cte_ratio | 0.2791 +- 0.0569 | 0.2180 +- 0.1258 |
| success_rate | 1.0000 | 0.5000 |

## Efficiency

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| duration_sec | 146.1366 +- 28.0312 | 161.0948 +- 41.1521 |
| approx_sample_rate_hz | 19.9999 +- 0.0010 | 20.0017 +- 0.0002 |
| mean_abs_cmd_linear | 0.5243 +- 0.0726 | 0.4553 +- 0.1029 |
| mean_abs_cmd_angular | 0.6943 +- 0.0403 | 0.5231 +- 0.1257 |
| rms_cmd_angular | 0.9143 +- 0.0614 | 0.6869 +- 0.1362 |

## Notes

- Lower error metrics are better.
- `success_rate` uses final goal distance <= 0.35 m.
- Efficiency here is execution efficiency from run traces (not CPU profiling).
