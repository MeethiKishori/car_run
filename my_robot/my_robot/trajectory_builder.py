#!/usr/bin/env python3

import argparse
import csv
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Sequence, Tuple


@dataclass
class Waypoint:
    x: float
    y: float
    v_hint: Optional[float]


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def load_waypoints(path: str) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            v_raw = (row.get("v_hint") or "").strip()
            v_hint = float(v_raw) if v_raw else None
            waypoints.append(Waypoint(x=x, y=y, v_hint=v_hint))

    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints to build a trajectory")

    # Remove accidental duplicates from repeated RViz clicks.
    deduped = [waypoints[0]]
    for w in waypoints[1:]:
        if _distance(deduped[-1].x, deduped[-1].y, w.x, w.y) > 1e-5:
            deduped.append(w)

    if len(deduped) < 2:
        raise ValueError("All waypoints are duplicates; need at least 2 unique points")
    return deduped


def cumulative_s(points: Sequence[Tuple[float, float]]) -> List[float]:
    s_vals = [0.0]
    for i in range(1, len(points)):
        s_vals.append(s_vals[-1] + _distance(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1]))
    return s_vals


def interpolate_on_polyline(
    points: Sequence[Tuple[float, float]],
    s_nodes: Sequence[float],
    sample_s: float,
) -> Tuple[float, float]:
    if sample_s <= s_nodes[0]:
        return points[0]
    if sample_s >= s_nodes[-1]:
        return points[-1]

    lo = 0
    hi = len(s_nodes) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if s_nodes[mid] <= sample_s:
            lo = mid
        else:
            hi = mid

    s0, s1 = s_nodes[lo], s_nodes[lo + 1]
    p0, p1 = points[lo], points[lo + 1]
    if s1 - s0 < 1e-9:
        return p0

    t = (sample_s - s0) / (s1 - s0)
    x = p0[0] + t * (p1[0] - p0[0])
    y = p0[1] + t * (p1[1] - p0[1])
    return x, y


def interpolate_scalar(
    values: Sequence[float],
    s_nodes: Sequence[float],
    sample_s: float,
) -> float:
    if sample_s <= s_nodes[0]:
        return values[0]
    if sample_s >= s_nodes[-1]:
        return values[-1]

    lo = 0
    hi = len(s_nodes) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if s_nodes[mid] <= sample_s:
            lo = mid
        else:
            hi = mid

    s0, s1 = s_nodes[lo], s_nodes[lo + 1]
    v0, v1 = values[lo], values[lo + 1]
    if s1 - s0 < 1e-9:
        return v0
    t = (sample_s - s0) / (s1 - s0)
    return v0 + t * (v1 - v0)


def build_reference_trajectory(
    waypoints: Sequence[Waypoint],
    spacing: float,
    v_max: float,
    a_lat_max: float,
    a_long_max: float,
) -> List[Tuple[int, float, float, float, float, float, float]]:
    base_points = [(w.x, w.y) for w in waypoints]
    s_nodes = cumulative_s(base_points)
    total_s = s_nodes[-1]
    if total_s < 1e-6:
        raise ValueError("Waypoint path length is too short")

    n_samples = max(2, int(total_s / spacing) + 1)
    s_samples = [min(i * spacing, total_s) for i in range(n_samples)]
    if s_samples[-1] < total_s:
        s_samples.append(total_s)

    xy_samples = [interpolate_on_polyline(base_points, s_nodes, s) for s in s_samples]

    # Build base speed profile from waypoint hints if present.
    if any(w.v_hint is not None for w in waypoints):
        hint_values = [w.v_hint if w.v_hint is not None else v_max for w in waypoints]
        base_speed = [interpolate_scalar(hint_values, s_nodes, s) for s in s_samples]
    else:
        base_speed = [v_max for _ in s_samples]

    yaws: List[float] = []
    for i in range(len(xy_samples)):
        if i < len(xy_samples) - 1:
            dx = xy_samples[i + 1][0] - xy_samples[i][0]
            dy = xy_samples[i + 1][1] - xy_samples[i][1]
        else:
            dx = xy_samples[i][0] - xy_samples[i - 1][0]
            dy = xy_samples[i][1] - xy_samples[i - 1][1]
        yaws.append(math.atan2(dy, dx))

    curvatures: List[float] = []
    ds = max(spacing, 1e-4)
    for i in range(len(xy_samples)):
        i_prev = max(i - 1, 0)
        i_next = min(i + 1, len(xy_samples) - 1)

        x_prev, y_prev = xy_samples[i_prev]
        x_curr, y_curr = xy_samples[i]
        x_next, y_next = xy_samples[i_next]

        dx = (x_next - x_prev) / (2.0 * ds)
        dy = (y_next - y_prev) / (2.0 * ds)
        ddx = (x_next - 2.0 * x_curr + x_prev) / (ds * ds)
        ddy = (y_next - 2.0 * y_curr + y_prev) / (ds * ds)

        denom = (dx * dx + dy * dy) ** 1.5
        if denom < 1e-9:
            kappa = 0.0
        else:
            kappa = (dx * ddy - dy * ddx) / denom
        curvatures.append(kappa)

    v_curve = []
    for k in curvatures:
        k_abs = abs(k)
        if k_abs < 1e-6:
            v_curve.append(v_max)
        else:
            v_curve.append(min(v_max, math.sqrt(max(a_lat_max, 1e-6) / k_abs)))

    v_ref = [max(0.05, min(base_speed[i], v_curve[i], v_max)) for i in range(len(s_samples))]

    # Forward acceleration limit.
    for i in range(1, len(v_ref)):
        ds_i = max(s_samples[i] - s_samples[i - 1], 1e-4)
        v_limit = math.sqrt(max(0.0, v_ref[i - 1] * v_ref[i - 1] + 2.0 * a_long_max * ds_i))
        v_ref[i] = min(v_ref[i], v_limit)

    # Backward braking limit.
    for i in range(len(v_ref) - 2, -1, -1):
        ds_i = max(s_samples[i + 1] - s_samples[i], 1e-4)
        v_limit = math.sqrt(max(0.0, v_ref[i + 1] * v_ref[i + 1] + 2.0 * a_long_max * ds_i))
        v_ref[i] = min(v_ref[i], v_limit)

    out: List[Tuple[int, float, float, float, float, float, float]] = []
    for i, s in enumerate(s_samples):
        x, y = xy_samples[i]
        out.append((i, s, x, y, yaws[i], curvatures[i], v_ref[i]))
    return out


def write_trajectory(path: str, rows: Sequence[Tuple[int, float, float, float, float, float, float]]) -> None:
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["idx", "s", "x", "y", "yaw", "kappa", "v_ref"])
        for idx, s, x, y, yaw, kappa, v_ref in rows:
            writer.writerow(
                [
                    idx,
                    f"{s:.6f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{yaw:.6f}",
                    f"{kappa:.6f}",
                    f"{v_ref:.6f}",
                ]
            )


def default_output_from_input(input_csv: str) -> str:
    base = os.path.basename(input_csv)
    stem, _ = os.path.splitext(base)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"/sim_ws/src/my_robot/maps/{stem}_traj_{ts}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build dense reference trajectory from waypoint CSV")
    parser.add_argument("--input", required=True, help="Input waypoint CSV (id,x,y,z,yaw,v_hint)")
    parser.add_argument("--output", default="", help="Output trajectory CSV path")
    parser.add_argument("--spacing", type=float, default=0.10, help="Trajectory spacing in meters")
    parser.add_argument("--v-max", type=float, default=0.80, help="Max reference speed (m/s)")
    parser.add_argument("--a-lat-max", type=float, default=1.50, help="Max lateral accel (m/s^2)")
    parser.add_argument("--a-long-max", type=float, default=1.00, help="Max longitudinal accel/decel (m/s^2)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_path = args.output if args.output else default_output_from_input(args.input)

    waypoints = load_waypoints(args.input)
    rows = build_reference_trajectory(
        waypoints=waypoints,
        spacing=max(args.spacing, 0.02),
        v_max=max(args.v_max, 0.1),
        a_lat_max=max(args.a_lat_max, 0.1),
        a_long_max=max(args.a_long_max, 0.1),
    )
    write_trajectory(out_path, rows)

    print(f"Input waypoints: {args.input}")
    print(f"Output trajectory: {out_path}")
    print(f"Trajectory points: {len(rows)}")
    print(
        f"Parameters: spacing={max(args.spacing, 0.02):.3f}, "
        f"v_max={max(args.v_max, 0.1):.3f}, "
        f"a_lat_max={max(args.a_lat_max, 0.1):.3f}, "
        f"a_long_max={max(args.a_long_max, 0.1):.3f}"
    )


if __name__ == "__main__":
    main()
