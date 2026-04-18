#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception as exc:
    raise RuntimeError(
        "matplotlib is required for graph generation. Install it in your active env."
    ) from exc


REQUIRED_COLS = [
    "timestamp_sec",
    "cmd_linear_x",
    "cmd_angular_z",
    "traj_cross_track_error",
    "traj_heading_error",
    "traj_goal_distance",
]

SUCCESS_GOAL_THRESHOLD = 0.35
HIGH_CTE_THRESHOLD = 0.35


@dataclass
class RunMetrics:
    controller: str
    file_name: str
    sample_count: int
    duration_sec: float
    approx_sample_rate_hz: float
    rmse_cte: float
    mae_cte: float
    p95_abs_cte: float
    max_abs_cte: float
    rmse_heading: float
    mae_heading: float
    final_goal_distance: float
    min_goal_distance: float
    high_cte_ratio: float
    mean_abs_cmd_linear: float
    mean_abs_cmd_angular: float
    rms_cmd_angular: float
    success: int


def _to_float(raw: str) -> float:
    raw = (raw or "").strip()
    if not raw:
        return math.nan
    try:
        return float(raw)
    except ValueError:
        return math.nan


def _load_array(path: str) -> Dict[str, np.ndarray]:
    cols = {k: [] for k in REQUIRED_COLS}

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing = [k for k in REQUIRED_COLS if k not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"Missing required columns in {path}: {missing}")

        for row in reader:
            vals = {k: _to_float(row.get(k, "")) for k in REQUIRED_COLS}
            if not all(math.isfinite(v) for v in vals.values()):
                continue
            for k, v in vals.items():
                cols[k].append(v)

    arr = {k: np.asarray(v, dtype=np.float64) for k, v in cols.items()}
    if arr["timestamp_sec"].size < 2:
        raise ValueError(f"Not enough valid rows in {path}")
    return arr


def _compute_metrics(controller: str, path: str, arr: Dict[str, np.ndarray]) -> Tuple[RunMetrics, np.ndarray, np.ndarray]:
    t = arr["timestamp_sec"]
    cmd_lin = arr["cmd_linear_x"]
    cmd_ang = arr["cmd_angular_z"]
    cte = arr["traj_cross_track_error"]
    heading = arr["traj_heading_error"]
    goal = arr["traj_goal_distance"]

    abs_cte = np.abs(cte)
    abs_heading = np.abs(heading)

    dt = np.diff(t)
    dt = dt[dt > 1e-6]
    duration = float(max(t[-1] - t[0], 0.0))
    sample_rate = float(1.0 / np.median(dt)) if dt.size > 0 else 0.0

    metrics = RunMetrics(
        controller=controller,
        file_name=os.path.basename(path),
        sample_count=int(t.size),
        duration_sec=duration,
        approx_sample_rate_hz=sample_rate,
        rmse_cte=float(np.sqrt(np.mean(np.square(cte)))),
        mae_cte=float(np.mean(abs_cte)),
        p95_abs_cte=float(np.percentile(abs_cte, 95)),
        max_abs_cte=float(np.max(abs_cte)),
        rmse_heading=float(np.sqrt(np.mean(np.square(heading)))),
        mae_heading=float(np.mean(abs_heading)),
        final_goal_distance=float(goal[-1]),
        min_goal_distance=float(np.min(goal)),
        high_cte_ratio=float(np.mean(abs_cte > HIGH_CTE_THRESHOLD)),
        mean_abs_cmd_linear=float(np.mean(np.abs(cmd_lin))),
        mean_abs_cmd_angular=float(np.mean(np.abs(cmd_ang))),
        rms_cmd_angular=float(np.sqrt(np.mean(np.square(cmd_ang)))),
        success=int(goal[-1] <= SUCCESS_GOAL_THRESHOLD),
    )

    progress = np.linspace(0.0, 1.0, t.size)
    return metrics, progress, abs_cte


def _write_metrics_csv(out_path: str, rows: List[RunMetrics]) -> None:
    fields = list(RunMetrics.__dataclass_fields__.keys())
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: getattr(r, k) for k in fields})


def _summary(rows: List[RunMetrics], key: str) -> Tuple[float, float]:
    vals = np.asarray([getattr(r, key) for r in rows], dtype=np.float64)
    if vals.size == 0:
        return math.nan, math.nan
    return float(np.mean(vals)), float(np.std(vals))


def _plot_summary_bar(out_path: str, pid_rows: List[RunMetrics], bc_rows: List[RunMetrics]) -> None:
    keys = ["rmse_cte", "rmse_heading", "final_goal_distance"]
    labels = ["RMSE CTE", "RMSE Heading", "Final Goal Dist"]

    pid_mean = [_summary(pid_rows, k)[0] for k in keys]
    pid_std = [_summary(pid_rows, k)[1] for k in keys]
    bc_mean = [_summary(bc_rows, k)[0] for k in keys]
    bc_std = [_summary(bc_rows, k)[1] for k in keys]

    x = np.arange(len(keys))
    w = 0.35

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(x - w / 2, pid_mean, w, yerr=pid_std, capsize=4, label="PID")
    ax.bar(x + w / 2, bc_mean, w, yerr=bc_std, capsize=4, label="BC")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Error (lower is better)")
    ax.set_title("Controller Accuracy Summary")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _plot_robustness_boxplots(out_path: str, pid_rows: List[RunMetrics], bc_rows: List[RunMetrics]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    pid_p95 = [r.p95_abs_cte for r in pid_rows]
    bc_p95 = [r.p95_abs_cte for r in bc_rows]
    axes[0].boxplot([pid_p95, bc_p95], labels=["PID", "BC"])
    axes[0].set_title("P95 |CTE| (Robustness)")
    axes[0].set_ylabel("Meters")
    axes[0].grid(axis="y", alpha=0.25)

    pid_max = [r.max_abs_cte for r in pid_rows]
    bc_max = [r.max_abs_cte for r in bc_rows]
    axes[1].boxplot([pid_max, bc_max], labels=["PID", "BC"])
    axes[1].set_title("Max |CTE| (Disturbance Sensitivity)")
    axes[1].set_ylabel("Meters")
    axes[1].grid(axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _resample_profile(progress: np.ndarray, abs_cte: np.ndarray, n: int = 100) -> np.ndarray:
    tgt = np.linspace(0.0, 1.0, n)
    return np.interp(tgt, progress, abs_cte)


def _plot_cte_progress_profile(
    out_path: str,
    pid_profiles: List[np.ndarray],
    bc_profiles: List[np.ndarray],
) -> None:
    x = np.linspace(0.0, 1.0, 100)

    pid_arr = np.vstack(pid_profiles)
    bc_arr = np.vstack(bc_profiles)

    pid_mean = pid_arr.mean(axis=0)
    pid_std = pid_arr.std(axis=0)
    bc_mean = bc_arr.mean(axis=0)
    bc_std = bc_arr.std(axis=0)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(x, pid_mean, label="PID mean |CTE|")
    ax.fill_between(x, pid_mean - pid_std, pid_mean + pid_std, alpha=0.2)
    ax.plot(x, bc_mean, label="BC mean |CTE|")
    ax.fill_between(x, bc_mean - bc_std, bc_mean + bc_std, alpha=0.2)
    ax.set_xlabel("Normalized trajectory progress")
    ax.set_ylabel("|Cross-track error| (m)")
    ax.set_title("Tracking Error Profile Over Trajectory")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _plot_efficiency_scatter(out_path: str, pid_rows: List[RunMetrics], bc_rows: List[RunMetrics]) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.scatter(
        [r.duration_sec for r in pid_rows],
        [r.final_goal_distance for r in pid_rows],
        label="PID",
        alpha=0.8,
    )
    ax.scatter(
        [r.duration_sec for r in bc_rows],
        [r.final_goal_distance for r in bc_rows],
        label="BC",
        alpha=0.8,
    )

    ax.set_xlabel("Run duration (s)")
    ax.set_ylabel("Final goal distance (m)")
    ax.set_title("Execution Efficiency: Time vs Final Error")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _write_report(path: str, pid_rows: List[RunMetrics], bc_rows: List[RunMetrics]) -> None:
    def line(metric: str) -> str:
        pid_m, pid_s = _summary(pid_rows, metric)
        bc_m, bc_s = _summary(bc_rows, metric)
        return f"| {metric} | {pid_m:.4f} +- {pid_s:.4f} | {bc_m:.4f} +- {bc_s:.4f} |"

    pid_success = np.mean([r.success for r in pid_rows]) if pid_rows else math.nan
    bc_success = np.mean([r.success for r in bc_rows]) if bc_rows else math.nan

    lines = [
        "# Controller Comparison Report",
        "",
        "## Inputs",
        "",
        f"- PID runs: {len(pid_rows)}",
        f"- BC runs: {len(bc_rows)}",
        "",
        "## Accuracy",
        "",
        "| Metric | PID (mean +- std) | BC (mean +- std) |",
        "|---|---:|---:|",
        line("rmse_cte"),
        line("mae_cte"),
        line("rmse_heading"),
        line("mae_heading"),
        line("final_goal_distance"),
        "",
        "## Robustness",
        "",
        "| Metric | PID (mean +- std) | BC (mean +- std) |",
        "|---|---:|---:|",
        line("p95_abs_cte"),
        line("max_abs_cte"),
        line("high_cte_ratio"),
        f"| success_rate | {pid_success:.4f} | {bc_success:.4f} |",
        "",
        "## Efficiency",
        "",
        "| Metric | PID (mean +- std) | BC (mean +- std) |",
        "|---|---:|---:|",
        line("duration_sec"),
        line("approx_sample_rate_hz"),
        line("mean_abs_cmd_linear"),
        line("mean_abs_cmd_angular"),
        line("rms_cmd_angular"),
        "",
        "## Notes",
        "",
        "- Lower error metrics are better.",
        "- `success_rate` uses final goal distance <= 0.35 m.",
        "- Efficiency here is execution efficiency from run traces (not CPU profiling).",
    ]

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def _collect_runs(datasets_dir: str, pattern: str, controller: str) -> Tuple[List[RunMetrics], List[np.ndarray]]:
    paths = sorted(glob.glob(os.path.join(datasets_dir, pattern)))
    if not paths:
        raise ValueError(f"No files matched for {controller}: {os.path.join(datasets_dir, pattern)}")

    rows: List[RunMetrics] = []
    profiles: List[np.ndarray] = []

    for path in paths:
        arr = _load_array(path)
        run, progress, abs_cte = _compute_metrics(controller, path, arr)
        rows.append(run)
        profiles.append(_resample_profile(progress, abs_cte))

    return rows, profiles


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare PID vs BC controller run datasets")
    parser.add_argument("--datasets-dir", required=True, help="Directory containing run CSV files")
    parser.add_argument("--pid-glob", default="pid_*.csv", help="Glob for PID run CSV files")
    parser.add_argument("--bc-glob", default="bc_*.csv", help="Glob for BC run CSV files")
    parser.add_argument("--out-dir", required=True, help="Output directory for report and graphs")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    pid_rows, pid_profiles = _collect_runs(args.datasets_dir, args.pid_glob, "PID")
    bc_rows, bc_profiles = _collect_runs(args.datasets_dir, args.bc_glob, "BC")

    all_rows = pid_rows + bc_rows
    _write_metrics_csv(os.path.join(args.out_dir, "controller_run_metrics.csv"), all_rows)

    _plot_summary_bar(os.path.join(args.out_dir, "summary_metrics_bar.png"), pid_rows, bc_rows)
    _plot_robustness_boxplots(os.path.join(args.out_dir, "robustness_boxplots.png"), pid_rows, bc_rows)
    _plot_cte_progress_profile(
        os.path.join(args.out_dir, "cte_progress_profile.png"),
        pid_profiles,
        bc_profiles,
    )
    _plot_efficiency_scatter(os.path.join(args.out_dir, "efficiency_scatter.png"), pid_rows, bc_rows)

    _write_report(
        os.path.join(args.out_dir, "controller_comparison_report.md"),
        pid_rows,
        bc_rows,
    )

    print("=== Controller comparison complete ===")
    print(f"Output directory: {args.out_dir}")
    print(f"PID files: {len(pid_rows)} | BC files: {len(bc_rows)}")


if __name__ == "__main__":
    main()
