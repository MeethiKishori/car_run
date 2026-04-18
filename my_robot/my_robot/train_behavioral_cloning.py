#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np


DEFAULT_FEATURES = [
    "odom_linear_x",
    "odom_angular_z",
    "scan_min",
    "scan_mean",
    "scan_std",
    "scan_front_min",
    "scan_left_min",
    "scan_right_min",
    "traj_cross_track_error",
    "traj_heading_error",
    "traj_goal_distance",
]

TARGETS = ["cmd_linear_x", "cmd_angular_z"]


def _to_float(row: Dict[str, str], key: str) -> float:
    raw = (row.get(key) or "").strip()
    if raw == "":
        return math.nan
    try:
        return float(raw)
    except ValueError:
        return math.nan


def load_dataset(csv_paths: List[str], feature_names: List[str]) -> Tuple[np.ndarray, np.ndarray, int]:
    xs: List[List[float]] = []
    ys: List[List[float]] = []
    dropped = 0

    for path in csv_paths:
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = [_to_float(row, k) for k in feature_names]
                y = [_to_float(row, k) for k in TARGETS]

                valid = all(math.isfinite(v) for v in x) and all(math.isfinite(v) for v in y)
                if not valid:
                    dropped += 1
                    continue

                xs.append(x)
                ys.append(y)

    if not xs:
        raise ValueError("No valid training rows after filtering NaN/Inf values")

    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64), dropped


def split_train_val(x: np.ndarray, y: np.ndarray, val_ratio: float, seed: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n = x.shape[0]
    idx = np.arange(n)
    rng = np.random.default_rng(seed)
    rng.shuffle(idx)

    n_val = max(1, int(n * val_ratio)) if n >= 5 else 0
    if n_val == 0:
        return x, y, x, y

    val_idx = idx[:n_val]
    train_idx = idx[n_val:]

    if len(train_idx) == 0:
        train_idx = val_idx

    return x[train_idx], y[train_idx], x[val_idx], y[val_idx]


def fit_ridge_multioutput(x: np.ndarray, y: np.ndarray, reg_lambda: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x_mean = np.mean(x, axis=0)
    x_std = np.std(x, axis=0)
    x_std = np.where(x_std < 1e-8, 1.0, x_std)

    y_mean = np.mean(y, axis=0)
    y_std = np.std(y, axis=0)
    y_std = np.where(y_std < 1e-8, 1.0, y_std)

    x_n = (x - x_mean) / x_std
    y_n = (y - y_mean) / y_std

    ones = np.ones((x_n.shape[0], 1), dtype=np.float64)
    x_aug = np.hstack([x_n, ones])

    feat_dim = x_aug.shape[1]
    reg = reg_lambda * np.eye(feat_dim, dtype=np.float64)
    reg[-1, -1] = 0.0  # don't regularize bias

    xtx = x_aug.T @ x_aug
    xty = x_aug.T @ y_n
    theta = np.linalg.solve(xtx + reg, xty)

    w = theta[:-1, :]  # [features, targets]
    b = theta[-1, :]   # [targets]

    return w, b, x_mean, x_std, y_mean, y_std


def predict(x: np.ndarray, w: np.ndarray, b: np.ndarray, x_mean: np.ndarray, x_std: np.ndarray, y_mean: np.ndarray, y_std: np.ndarray) -> np.ndarray:
    x_n = (x - x_mean) / x_std
    y_n = x_n @ w + b
    return y_n * y_std + y_mean


def metrics(y_true: np.ndarray, y_pred: np.ndarray) -> Dict[str, float]:
    err = y_pred - y_true
    mse = np.mean(np.square(err), axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(err), axis=0)

    y_mean = np.mean(y_true, axis=0)
    ss_res = np.sum(np.square(y_true - y_pred), axis=0)
    ss_tot = np.sum(np.square(y_true - y_mean), axis=0)
    r2 = 1.0 - np.divide(ss_res, np.where(ss_tot < 1e-12, 1.0, ss_tot))

    return {
        "mae_linear": float(mae[0]),
        "mae_angular": float(mae[1]),
        "rmse_linear": float(rmse[0]),
        "rmse_angular": float(rmse[1]),
        "r2_linear": float(r2[0]),
        "r2_angular": float(r2[1]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a simple behavioral-cloning controller (ridge regression)")
    parser.add_argument("--input", nargs="+", required=True, help="Input dataset CSV file(s)")
    parser.add_argument("--output", required=False, default="", help="Output model JSON path")
    parser.add_argument("--reg-lambda", type=float, default=1e-3, help="Ridge regularization")
    parser.add_argument("--val-ratio", type=float, default=0.2, help="Validation split ratio")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    for p in args.input:
        if not os.path.exists(p):
            raise FileNotFoundError(f"Dataset file not found: {p}")

    out_path = args.output
    if not out_path:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = f"/sim_ws/src/my_robot/models/bc_model_{ts}.json"

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    x, y, dropped = load_dataset(args.input, DEFAULT_FEATURES)
    x_train, y_train, x_val, y_val = split_train_val(x, y, args.val_ratio, args.seed)

    w, b, x_mean, x_std, y_mean, y_std = fit_ridge_multioutput(x_train, y_train, args.reg_lambda)

    train_pred = predict(x_train, w, b, x_mean, x_std, y_mean, y_std)
    val_pred = predict(x_val, w, b, x_mean, x_std, y_mean, y_std)

    train_m = metrics(y_train, train_pred)
    val_m = metrics(y_val, val_pred)

    model = {
        "model_type": "behavioral_cloning_ridge_v1",
        "feature_names": DEFAULT_FEATURES,
        "target_names": TARGETS,
        "weights": w.tolist(),
        "bias": b.tolist(),
        "x_mean": x_mean.tolist(),
        "x_std": x_std.tolist(),
        "y_mean": y_mean.tolist(),
        "y_std": y_std.tolist(),
        "reg_lambda": args.reg_lambda,
        "rows_total_valid": int(x.shape[0]),
        "rows_dropped": int(dropped),
        "rows_train": int(x_train.shape[0]),
        "rows_val": int(x_val.shape[0]),
        "metrics_train": train_m,
        "metrics_val": val_m,
        "input_files": args.input,
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(model, f, indent=2)

    print("=== Behavioral Cloning Training Complete ===")
    print(f"Output model: {out_path}")
    print(f"Rows valid: {x.shape[0]} | dropped: {dropped}")
    print(f"Train rows: {x_train.shape[0]} | Val rows: {x_val.shape[0]}")
    print("Train metrics:")
    print(json.dumps(train_m, indent=2))
    print("Validation metrics:")
    print(json.dumps(val_m, indent=2))


if __name__ == "__main__":
    main()
