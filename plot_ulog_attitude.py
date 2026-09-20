#!/usr/bin/env python3
"""
plot_ulog_attitude.py
----------------------
Loads a PX4 .ulg flight log (from QGC's Log Download, or straight off SITL's
rootfs/log/ folder) and plots the actual attitude quaternion (vehicle_attitude)
against the commanded/reference quaternion (vehicle_attitude_setpoint) -- the
same actual-vs-reference comparison as the earlier MAVSDK step test, but
sourced from a real flight/mission log instead of a scripted Offboard test.

Requirements:
    pip install pyulog numpy matplotlib

Usage:
    # Whole flight
    python plot_ulog_attitude.py --log my_flight.ulg

    # Zoom into a specific maneuver (e.g. the pitch step within the mission),
    # in seconds since the log started
    python plot_ulog_attitude.py --log my_flight.ulg --start-s 120 --end-s 135

Tip: if you don't know when the maneuver happened, run once with no
--start-s/--end-s to get an overview plot and console-printed time range,
then re-run zoomed into the interesting part. You can also eyeball timing on
https://review.px4.io by uploading the same .ulg first.
"""

import argparse
import csv
import math

import numpy as np
from pyulog import ULog


# --------------------------------------------------------------------------- #
# Quaternion helpers (same convention/formulas as the flight-test script)
# --------------------------------------------------------------------------- #

def quat_mul(a, b):
    """Hamilton product a (x) b, both as (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_inv(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_geodesic_deg(q_from, q_to) -> float:
    """Geodesic angle (deg) on S^3, theta_e = 2*arccos(|q_e0|), q_e = q_to (x) q_from^-1."""
    q_e = quat_mul(q_to, quat_inv(q_from))
    return math.degrees(2.0 * math.acos(float(np.clip(abs(q_e[0]), -1.0, 1.0))))


# --------------------------------------------------------------------------- #
# ULog loading
# --------------------------------------------------------------------------- #

def load_quaternion_topic(ulog: ULog, topic: str, field_prefix: str, t0_us: int):
    """
    Returns (t_seconds, quat_array[N,4]) for a uORB topic containing a
    quaternion field like q[0..3] or q_d[0..3], time-shifted so t=0 is t0_us.
    """
    dataset = ulog.get_dataset(topic)
    d = dataset.data
    t = (np.array(d["timestamp"], dtype=np.float64) - t0_us) / 1e6
    q = np.stack([
        np.array(d[f"{field_prefix}[0]"]),
        np.array(d[f"{field_prefix}[1]"]),
        np.array(d[f"{field_prefix}[2]"]),
        np.array(d[f"{field_prefix}[3]"]),
    ], axis=1)
    return t, q


def nearest_idx(t_array: np.ndarray, t: float) -> int:
    return int(np.argmin(np.abs(t_array - t)))


# --------------------------------------------------------------------------- #
# Plot / CSV / metrics
# --------------------------------------------------------------------------- #

def window_mask(t: np.ndarray, start_s, end_s) -> np.ndarray:
    mask = np.ones_like(t, dtype=bool)
    if start_s is not None:
        mask &= t >= start_s
    if end_s is not None:
        mask &= t <= end_s
    return mask


def plot_attitude(t_act, q_act, t_ref, q_ref, out_path: str, title_suffix: str = ""):
    import matplotlib.pyplot as plt

    labels = ["w", "x", "y", "z"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t_ref, q_ref[:, i], color="tab:orange", linewidth=1.2,
                label="reference (vehicle_attitude_setpoint)" if i == 0 else None)
        ax.plot(t_act, q_act[:, i], color="tab:blue", linewidth=1.0,
                label="actual (vehicle_attitude)" if i == 0 else None)
        ax.set_ylabel(f"q_{labels[i]}")
        ax.grid(True, alpha=0.3)
    axes[0].set_title(f"Attitude quaternion: actual vs reference{title_suffix}")
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time since log start [s]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"-- Saved plot to {out_path}")


def save_csv(t_act, q_act, t_ref, q_ref, out_path: str):
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "qw_act", "qx_act", "qy_act", "qz_act",
                          "qw_ref", "qx_ref", "qy_ref", "qz_ref"])
        for i, t in enumerate(t_act):
            j = nearest_idx(t_ref, t)
            writer.writerow([t, *q_act[i], *q_ref[j]])
    print(f"-- Wrote {out_path}")


def compute_rmse(t_act, q_act, t_ref, q_ref) -> float:
    errs = []
    for i, t in enumerate(t_act):
        j = nearest_idx(t_ref, t)
        errs.append(quat_geodesic_deg(q_act[i], q_ref[j]))
    return float(np.sqrt(np.mean(np.array(errs) ** 2)))


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main():
    p = argparse.ArgumentParser(description="Plot actual vs. reference attitude quaternion from a PX4 .ulg log")
    p.add_argument("--log", required=True, help="Path to the .ulg file")
    p.add_argument("--start-s", type=float, default=None,
                    help="Start of the window to plot/analyze, seconds since log start (default: full log)")
    p.add_argument("--end-s", type=float, default=None,
                    help="End of the window to plot/analyze, seconds since log start (default: full log)")
    p.add_argument("--csv", default="ulog_attitude.csv", help="Output CSV path")
    p.add_argument("--plot", default="ulog_attitude_plot.png", help="Output plot PNG path")
    args = p.parse_args()

    print(f"-- Loading {args.log}")
    ulog = ULog(args.log, message_name_filter_list=["vehicle_attitude", "vehicle_attitude_setpoint"])

    t0_us = ulog.start_timestamp
    t_act, q_act = load_quaternion_topic(ulog, "vehicle_attitude", "q", t0_us)
    t_ref, q_ref = load_quaternion_topic(ulog, "vehicle_attitude_setpoint", "q_d", t0_us)

    print(f"-- Log spans {t_act[0]:.1f}s to {t_act[-1]:.1f}s "
          f"({len(t_act)} attitude samples, {len(t_ref)} setpoint samples)")

    m_act = window_mask(t_act, args.start_s, args.end_s)
    m_ref = window_mask(t_ref, args.start_s, args.end_s)
    t_act_w, q_act_w = t_act[m_act], q_act[m_act]
    t_ref_w, q_ref_w = t_ref[m_ref], q_ref[m_ref]

    if len(t_act_w) < 2 or len(t_ref_w) < 2:
        print("-- Window contains too little data (check --start-s/--end-s); "
              "falling back to the full log.")
        t_act_w, q_act_w = t_act, q_act
        t_ref_w, q_ref_w = t_ref, q_ref

    title_suffix = f" ({args.start_s or 0:.1f}s - {args.end_s if args.end_s is not None else t_act_w[-1]:.1f}s)" \
        if (args.start_s is not None or args.end_s is not None) else ""

    save_csv(t_act_w, q_act_w, t_ref_w, q_ref_w, args.csv)
    plot_attitude(t_act_w, q_act_w, t_ref_w, q_ref_w, args.plot, title_suffix)

    rmse = compute_rmse(t_act_w, q_act_w, t_ref_w, q_ref_w)
    print(f"-- RMSE_theta over this window: {rmse:.2f} deg")


if __name__ == "__main__":
    main()
