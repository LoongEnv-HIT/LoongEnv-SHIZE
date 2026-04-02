from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tempfile

import numpy as np

from .pipeline import (
    PathPreparationConfig,
    PipelineResult,
    RuckigConfig,
    ToppraConfig,
    load_joint_waypoints,
    run_mvc_ruckig_pipeline,
)


@dataclass
class CorePipelineConfig:
    resample_points: int = 1000
    smooth_passes: int = 60
    smooth_blend: float = 0.49
    dt: float = 0.001
    v_max: float = 0.02
    a_max: float = 0.02
    j_max: float = 0.04
    intermediate_velocity: float = 0.005
    minima_count: int | None = None
    minima_gap: int = 12
    minima_threshold_mode: str = "mean"
    minima_max_value: float | None = None
    junction_velocity_scale: float = 0.8


@dataclass
class CorePipelineResult:
    ts: np.ndarray
    qs: np.ndarray
    qds: np.ndarray
    qdds: np.ndarray
    mvc_upper: np.ndarray
    minima_idx: np.ndarray
    minima_v: np.ndarray
    segment_points: list[int]
    segment_targets: list[float]
    segment_target_velocities: np.ndarray
    gridpoints: np.ndarray
    ss: np.ndarray
    waypoints_raw: np.ndarray
    waypoints_smooth: np.ndarray
    pipeline: PipelineResult


def _to_waypoint_array(waypoints_or_path) -> np.ndarray:
    if isinstance(waypoints_or_path, (str, Path)):
        return load_joint_waypoints(str(waypoints_or_path), dtype=np.float64)
    return np.asarray(waypoints_or_path, dtype=np.float64)


def run_core_pipeline(
    robot_model,
    waypoints_or_path,
    vlim: np.ndarray,
    alim: np.ndarray,
    config: CorePipelineConfig | None = None,
    *,
    swap_xy_for_ee: bool = False,
) -> CorePipelineResult:
    if config is None:
        config = CorePipelineConfig()

    waypoints = _to_waypoint_array(waypoints_or_path)
    tmp_path: Path | None = None
    if not isinstance(waypoints_or_path, (str, Path)):
        with tempfile.NamedTemporaryFile(
            prefix="fixed_toppra_mvc_ruckig_", suffix=".txt", delete=False
        ) as f:
            tmp_path = Path(f.name)
        np.savetxt(tmp_path, waypoints, fmt="%.12f")
        input_path = str(tmp_path)
    else:
        input_path = str(waypoints_or_path)

    try:
        pipeline = run_mvc_ruckig_pipeline(
            robot_model=robot_model,
            vlim=np.asarray(vlim, dtype=float),
            alim=np.asarray(alim, dtype=float),
            path_config=PathPreparationConfig(
                input_path=input_path,
                resample_points=int(config.resample_points),
                joint_smooth_passes=int(config.smooth_passes),
                joint_smooth_blend=float(config.smooth_blend),
                swap_xy_for_ee=bool(swap_xy_for_ee),
            ),
            toppra_config=ToppraConfig(),
            ruckig_config=RuckigConfig(
                dt=float(config.dt),
                v_max=float(config.v_max),
                a_max=float(config.a_max),
                j_max=float(config.j_max),
                intermediate_velocity=float(config.intermediate_velocity),
                minima_count=(None if config.minima_count is None else int(config.minima_count)),
                minima_gap=int(config.minima_gap),
                minima_threshold_mode=str(config.minima_threshold_mode),
                minima_max_value=(
                    None if config.minima_max_value is None else float(config.minima_max_value)
                ),
                junction_velocity_scale=float(config.junction_velocity_scale),
            ),
        )
    finally:
        if tmp_path is not None and tmp_path.exists():
            tmp_path.unlink()

    return CorePipelineResult(
        ts=pipeline.ts,
        qs=pipeline.qs,
        qds=pipeline.qds,
        qdds=pipeline.qdds,
        mvc_upper=pipeline.mvc_upper,
        minima_idx=pipeline.minima_idx,
        minima_v=pipeline.minima_v,
        segment_points=pipeline.segment_points,
        segment_targets=pipeline.segment_targets,
        segment_target_velocities=pipeline.segment_target_velocities,
        gridpoints=pipeline.prepared.gridpoints,
        ss=pipeline.prepared.ss,
        waypoints_raw=pipeline.prepared.waypoints_raw,
        waypoints_smooth=pipeline.prepared.waypoints_smooth,
        pipeline=pipeline,
    )
