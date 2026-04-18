# Controller Comparison Report

## Inputs

- PID runs: 3
- BC runs: 3

## Accuracy

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| rmse_cte | 0.3147 +- 0.0529 | 0.3331 +- 0.1001 |
| mae_cte | 0.2281 +- 0.0372 | 0.2431 +- 0.0771 |
| rmse_heading | 0.6129 +- 0.1878 | 0.4322 +- 0.0764 |
| mae_heading | 0.4071 +- 0.1112 | 0.3160 +- 0.0585 |
| final_goal_distance | 0.0694 +- 0.0132 | 1.1688 +- 0.7628 |

## Robustness

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| p95_abs_cte | 0.6223 +- 0.0632 | 0.6965 +- 0.1892 |
| max_abs_cte | 0.7561 +- 0.1038 | 0.8106 +- 0.2181 |
| high_cte_ratio | 0.2725 +- 0.0474 | 0.2314 +- 0.1045 |
| success_rate | 1.0000 | 0.3333 |

## Efficiency

| Metric | PID (mean +- std) | BC (mean +- std) |
|---|---:|---:|
| duration_sec | 141.8311 +- 23.6835 | 151.5256 +- 36.2235 |
| approx_sample_rate_hz | 20.0008 +- 0.0015 | 20.0002 +- 0.0020 |
| mean_abs_cmd_linear | 0.5451 +- 0.0662 | 0.4674 +- 0.0857 |
| mean_abs_cmd_angular | 0.6887 +- 0.0338 | 0.5435 +- 0.1066 |
| rms_cmd_angular | 0.9077 +- 0.0510 | 0.7162 +- 0.1187 |

## Notes

- Lower error metrics are better.
- `success_rate` uses final goal distance <= 0.35 m.
- Efficiency here is execution efficiency from run traces (not CPU profiling).
