# Controller Comparison Report

## Inputs

- PID runs: 1
- BC runs: 1

## Accuracy

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| rmse_cte | 0.3894 +- 0.0000 | 0.1932 +- 0.0000 |
| mae_cte | 0.2805 +- 0.0000 | 0.1473 +- 0.0000 |
| rmse_heading | 0.8765 +- 0.0000 | 0.3560 +- 0.0000 |
| mae_heading | 0.5615 +- 0.0000 | 0.2411 +- 0.0000 |
| final_goal_distance | 0.0583 +- 0.0000 | 0.2980 +- 0.0000 |

## Robustness

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| p95_abs_cte | 0.7040 +- 0.0000 | 0.4293 +- 0.0000 |
| max_abs_cte | 0.8361 +- 0.0000 | 0.5111 +- 0.0000 |
| high_cte_ratio | 0.3360 +- 0.0000 | 0.0922 +- 0.0000 |
| success_rate | 1.0000 | 1.0000 |

## Efficiency

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| duration_sec | 174.1678 +- 0.0000 | 119.9427 +- 0.0000 |
| approx_sample_rate_hz | 19.9990 +- 0.0000 | 20.0014 +- 0.0000 |
| mean_abs_cmd_linear | 0.4517 +- 0.0000 | 0.5581 +- 0.0000 |
| mean_abs_cmd_angular | 0.7345 +- 0.0000 | 0.3973 +- 0.0000 |
| rms_cmd_angular | 0.9757 +- 0.0000 | 0.5507 +- 0.0000 |

## Notes

- Lower error metrics are better.
- `success_rate` uses final goal distance <= 0.35 m.
- Efficiency here is execution efficiency from run traces (not CPU profiling).
