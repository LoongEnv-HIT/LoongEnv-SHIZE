from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .core import CorePipelineConfig, run_core_pipeline
from .pipeline import resample_traj_uniform


def _load_robot(spec: str):
    import roboticstoolbox as rtb
    import importlib

    if ":" in spec:
        module_name, attr_name = spec.split(":", 1)
        module = importlib.import_module(module_name)
        factory = getattr(module, attr_name)
    else:
        factory = getattr(rtb.models.DH, spec)
    return factory()


def _build_limits(values: list[float]) -> np.ndarray:
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.shape[0] != 6:
        raise ValueError("Expected 6 values for joint limits.")
    return np.column_stack((-arr, arr))


def _finite_mean(values: np.ndarray) -> float | None:
    arr = np.asarray(values, dtype=float).reshape(-1)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return None
    return float(np.mean(finite))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Smooth a joint path, compute MVC with TOPPRA, decelerate at the selected MVC minima, "
            "and run segmented 1-DoF Ruckig with each minima's MVC speed as the segment junction velocity."
        )
    )
    parser.add_argument("--input", default="traj_seg_001.txt", help="Input trajectory file with 6 joint columns.")
    parser.add_argument("--robot", default="hsr650", help="Robot factory name in roboticstoolbox.models.DH or module:callable.")
    parser.add_argument(
        "--vmax-joints",
        nargs=6,
        type=float,
        default=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Per-joint symmetric velocity limits in rad/s.",
    )
    parser.add_argument(
        "--amax-joints",
        nargs=6,
        type=float,
        default=[10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Per-joint symmetric acceleration limits in rad/s^2.",
    )
    parser.add_argument("--resample-points", type=int, default=1000)
    parser.add_argument("--smooth-passes", type=int, default=60)
    parser.add_argument("--smooth-blend", type=float, default=0.49)
    parser.add_argument("--dt", type=float, default=0.001, help="Ruckig path-parameter timestep in seconds.")
    parser.add_argument("--v-max", type=float, default=0.02, help="1-DoF Ruckig max velocity on s.")
    parser.add_argument("--a-max", type=float, default=0.02, help="1-DoF Ruckig max acceleration on s.")
    parser.add_argument("--j-max", type=float, default=0.04, help="1-DoF Ruckig max jerk on s.")
    parser.add_argument(
        "--intermediate-velocity",
        type=float,
        default=0.005,
        help="Legacy compatibility knob. Junction velocities now come from the selected MVC minima.",
    )
    parser.add_argument("--minima-count", type=int, default=0, help="Number of MVC minima to keep. Use 0 for all valid minima.")
    parser.add_argument("--minima-gap", type=int, default=12)
    parser.add_argument(
        "--minima-threshold-mode",
        choices=["mean", "fixed", "none"],
        default="mean",
        help="How to threshold minima before segmentation.",
    )
    parser.add_argument(
        "--minima-max-value",
        type=float,
        default=-1.0,
        help="Upper bound used only when --minima-threshold-mode=fixed. Use a negative value to disable the fixed threshold.",
    )
    parser.add_argument(
        "--junction-velocity-scale",
        type=float,
        default=0.8,
        help="Scale factor applied to each selected minima MVC speed to get the Ruckig junction speed.",
    )
    parser.add_argument("--swap-xy-for-ee", action="store_true", help="Swap FK x/y when parameterizing by EE arc length.")
    parser.add_argument("--export-dt-ms", type=float, default=4.0, help="Export output joint trajectory at uniform dt in ms.")
    parser.add_argument(
        "--output-dir",
        default="outputs/fixed_toppra_mvc_ruckig",
        help="Directory for generated outputs.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = _load_robot(args.robot)
    vlim = _build_limits(args.vmax_joints)
    alim = _build_limits(args.amax_joints)
    config = CorePipelineConfig(
        resample_points=int(args.resample_points),
        smooth_passes=int(args.smooth_passes),
        smooth_blend=float(args.smooth_blend),
        dt=float(args.dt),
        v_max=float(args.v_max),
        a_max=float(args.a_max),
        j_max=float(args.j_max),
        intermediate_velocity=float(args.intermediate_velocity),
        minima_count=(None if int(args.minima_count) <= 0 else int(args.minima_count)),
        minima_gap=int(args.minima_gap),
        minima_threshold_mode=str(args.minima_threshold_mode),
        minima_max_value=(None if float(args.minima_max_value) < 0 else float(args.minima_max_value)),
        junction_velocity_scale=float(args.junction_velocity_scale),
    )
    result = run_core_pipeline(
        robot_model=robot,
        waypoints_or_path=args.input,
        vlim=vlim,
        alim=alim,
        config=config,
        swap_xy_for_ee=bool(args.swap_xy_for_ee),
    )

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    np.savetxt(out_dir / "joint_position.txt", result.qs, fmt="%.9f")
    np.savetxt(out_dir / "joint_velocity.txt", result.qds, fmt="%.9f")
    np.savetxt(out_dir / "joint_acceleration.txt", result.qdds, fmt="%.9f")
    np.savetxt(out_dir / "mvc_upper.txt", result.mvc_upper, fmt="%.9f")

    export_dt = float(args.export_dt_ms) * 1e-3
    t_uniform, q_uniform = resample_traj_uniform(result.ts, result.qs, export_dt)
    np.savetxt(out_dir / "joint_position_uniform.txt", q_uniform, fmt="%.9f")

    profile = np.column_stack((result.pipeline.ts, result.pipeline.s_path, result.pipeline.sd_path, result.pipeline.sdd_path))
    np.savetxt(out_dir / "path_profile.csv", profile, fmt="%.9f", delimiter=",", header="t,s,sd,sdd", comments="")

    summary = {
        "input": str(args.input),
        "robot": str(args.robot),
        "duration": float(result.ts[-1]),
        "samples": int(result.qs.shape[0]),
        "uniform_export_dt_s": export_dt,
        "uniform_export_samples": int(q_uniform.shape[0]),
        "minima_threshold_mode": str(args.minima_threshold_mode),
        "minima_threshold_value": (
            _finite_mean(result.mvc_upper)
            if str(args.minima_threshold_mode) == "mean"
            else (None if float(args.minima_max_value) < 0 else float(args.minima_max_value))
        ),
        "minima_max_value": (None if float(args.minima_max_value) < 0 else float(args.minima_max_value)),
        "minima_idx": result.minima_idx.astype(int).tolist(),
        "minima_v": result.minima_v.astype(float).tolist(),
        "segment_points": [int(v) for v in result.segment_points],
        "segment_targets": [float(v) for v in result.segment_targets],
        "junction_velocity_scale": float(args.junction_velocity_scale),
        "segment_target_velocities": result.segment_target_velocities.astype(float).tolist(),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"[fixed-toppra-mvc-ruckig] output_dir={out_dir}")
    print(f"[fixed-toppra-mvc-ruckig] duration={result.ts[-1]:.6f}s samples={result.qs.shape[0]}")
    print(f"[fixed-toppra-mvc-ruckig] minima_idx={summary['minima_idx']}")


if __name__ == "__main__":
    main()
