from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import traceback
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PACKAGE_ROOT = PROJECT_ROOT / 'fixed_toppra_mvc_ruckig'
for candidate in (PROJECT_ROOT, PACKAGE_ROOT):
    candidate_str = str(candidate)
    if candidate.exists() and candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)

from fixed_toppra_mvc_ruckig import (
    PathPreparationConfig,
    RuckigConfig,
    ToppraConfig,
    resample_traj_uniform,
    run_mvc_ruckig_pipeline,
)


def _quat_wxyz_to_rotation(quat_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = quat_wxyz
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return np.eye(3, dtype=float)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _make_transform(pos: tuple[float, float, float], quat_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = _quat_wxyz_to_rotation(quat_wxyz)
    t[:3, 3] = np.asarray(pos, dtype=float)
    return t


def build_er15_robot_model():
    import roboticstoolbox as rtb

    limits = [
        (-2.967, 2.967),
        (-2.7925, 1.5708),
        (-1.4835, 3.0543),
        (-3.316, 3.316),
        (-2.2689, 2.2689),
        (-6.2832, 6.2832),
    ]

    transforms = [
        ((0.0, 0.0, 0.43), (1.0, 0.0, 0.0, 0.0)),
        ((0.18, 0.0, 0.0), (0.5, 0.5, -0.5, 0.5)),
        ((0.58, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        ((0.16, -0.64, 0.0), (2.08523e-10, -2.08523e-10, -0.707107, 0.707107)),
        ((0.0, 0.0, 0.0), (2.08523e-10, -2.08523e-10, -0.707107, 0.707107)),
        ((0.0, -0.116, 0.0), (0.707107, 0.707107, 0.0, 0.0)),
    ]

    links = []
    parent = None

    for index, (transform, qlim) in enumerate(zip(transforms, limits), start=1):
        pos, quat = transform
        fixed_t = _make_transform(pos, quat)
        ets = rtb.ET.SE3(fixed_t) * rtb.ET.Rz()
        link = rtb.Link(
            ets,
            parent=parent,
            name=f"link_{index}",
            qlim=np.asarray(qlim, dtype=float),
        )
        links.append(link)
        parent = link

    return rtb.Robot(links, name='ER15-1400', manufacturer='LoongEnv')


def _validate_and_normalize_limits(raw: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    velocity = np.asarray(raw.get('velocity', [1.0] * 6), dtype=float).reshape(-1)
    acceleration = np.asarray(raw.get('acceleration', [10.0] * 6), dtype=float).reshape(-1)
    if velocity.shape[0] != 6 or acceleration.shape[0] != 6:
        raise ValueError('limits.velocity and limits.acceleration must each contain 6 values.')
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError('velocity and acceleration limits must be positive.')
    vlim = np.column_stack((-velocity, velocity))
    alim = np.column_stack((-acceleration, acceleration))
    return vlim, alim


def _downsample_series(xs: np.ndarray, ys: np.ndarray, max_points: int = 500) -> list[dict[str, float]]:
    xs = np.asarray(xs, dtype=float).reshape(-1)
    ys = np.asarray(ys, dtype=float).reshape(-1)
    n = min(xs.shape[0], ys.shape[0])
    if n == 0:
        return []
    if n <= max_points:
        return [{'x': float(xs[i]), 'y': float(ys[i])} for i in range(n)]
    step = max(1, int(math.ceil(n / max_points)))
    sampled = [{'x': float(xs[i]), 'y': float(ys[i])} for i in range(0, n, step)]
    if sampled[-1]['x'] != float(xs[n - 1]):
        sampled.append({'x': float(xs[n - 1]), 'y': float(ys[n - 1])})
    return sampled


def _downsample_profile(ts: np.ndarray, s: np.ndarray, sd: np.ndarray, sdd: np.ndarray, max_points: int = 500) -> list[dict[str, float]]:
    n = min(len(ts), len(s), len(sd), len(sdd))
    if n == 0:
        return []
    if n <= max_points:
        return [
            {'t': float(ts[i]), 's': float(s[i]), 'sd': float(sd[i]), 'sdd': float(sdd[i])}
            for i in range(n)
        ]
    step = max(1, int(math.ceil(n / max_points)))
    sampled = [
        {'t': float(ts[i]), 's': float(s[i]), 'sd': float(sd[i]), 'sdd': float(sdd[i])}
        for i in range(0, n, step)
    ]
    if sampled[-1]['t'] != float(ts[n - 1]):
        sampled.append(
            {'t': float(ts[n - 1]), 's': float(s[n - 1]), 'sd': float(sd[n - 1]), 'sdd': float(sdd[n - 1])}
        )
    return sampled


def _planner_defaults() -> dict[str, float | int | str | None]:
    return {
        'resample_points': 1000,
        'smooth_passes': 60,
        'smooth_blend': 0.49,
        'dt': 0.001,
        'v_max': 0.02,
        'a_max': 0.02,
        'j_max': 0.04,
        'minima_count': None,
        'minima_gap': 12,
        'minima_threshold_mode': 'mean',
        'minima_max_value': None,
        'junction_velocity_scale': 0.8,
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        fv = float(value)
        return fv if math.isfinite(fv) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def _health() -> dict[str, Any]:
    deps = ['numpy', 'pandas', 'scipy', 'toppra', 'ecos', 'ruckig', 'roboticstoolbox', 'spatialmath']
    dep_state = {name: bool(importlib.util.find_spec(name)) for name in deps}

    er15_ok = False
    er15_error = None
    try:
        robot = build_er15_robot_model()
        _ = robot.fkine(np.zeros(6, dtype=float))
        er15_ok = True
    except Exception as error:  # pragma: no cover - diagnostics path
        er15_error = str(error)

    return {
        'ok': True,
        'python': {
            'executable': sys.executable,
            'version': sys.version,
        },
        'dependencies': dep_state,
        'er15Factory': {
            'ok': er15_ok,
            'error': er15_error,
        },
    }


def _run(payload: dict[str, Any]) -> dict[str, Any]:
    waypoint_text = str(payload.get('waypointText', '')).strip()
    waypoint_text = waypoint_text.lstrip('\ufeff')
    if not waypoint_text:
        raise ValueError('waypointText is required.')

    file_name = str(payload.get('fileName') or 'waypoints.csv')
    suffix = Path(file_name).suffix or '.csv'

    vlim, alim = _validate_and_normalize_limits(payload.get('limits', {}))

    requested_config = payload.get('config', {}) or {}
    defaults = _planner_defaults()
    merged_config = {**defaults, **requested_config}

    ss_mode = str(payload.get('ssMode') or 'arc').strip().lower()
    if ss_mode not in {'arc', 'time'}:
        raise ValueError("ssMode must be 'arc' or 'time'.")

    playback_dt_ms = float(payload.get('playbackDtMs', 4.0))
    if playback_dt_ms <= 0:
        raise ValueError('playbackDtMs must be > 0.')

    logs: list[str] = []
    warnings: list[str] = []

    robot = build_er15_robot_model()
    _ = robot.fkine(np.zeros(6, dtype=float))

    tmp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(prefix='er15_waypoints_', suffix=suffix, delete=False, mode='w', encoding='utf-8') as f:
            f.write(waypoint_text)
            tmp_path = Path(f.name)

        path_config = PathPreparationConfig(
            input_path=str(tmp_path),
            ss_mode=ss_mode,
            resample_points=int(merged_config['resample_points']),
            joint_smooth_passes=int(merged_config['smooth_passes']),
            joint_smooth_blend=float(merged_config['smooth_blend']),
            swap_xy_for_ee=False,
        )
        toppra_config = ToppraConfig()
        ruckig_config = RuckigConfig(
            dt=float(merged_config['dt']),
            v_max=float(merged_config['v_max']),
            a_max=float(merged_config['a_max']),
            j_max=float(merged_config['j_max']),
            minima_count=(None if merged_config['minima_count'] in (None, '', 0) else int(merged_config['minima_count'])),
            minima_gap=int(merged_config['minima_gap']),
            minima_threshold_mode=str(merged_config['minima_threshold_mode']),
            minima_max_value=(
                None if merged_config['minima_max_value'] in (None, '') else float(merged_config['minima_max_value'])
            ),
            junction_velocity_scale=float(merged_config['junction_velocity_scale']),
        )

        result = run_mvc_ruckig_pipeline(
            robot_model=robot,
            vlim=vlim,
            alim=alim,
            path_config=path_config,
            toppra_config=toppra_config,
            ruckig_config=ruckig_config,
        )

        uniform_dt_s = playback_dt_ms * 1e-3
        t_uniform, q_uniform = resample_traj_uniform(result.ts, result.qs, uniform_dt_s)

        summary = {
            'inputFileName': file_name,
            'duration': float(result.ts[-1]),
            'samples': int(result.qs.shape[0]),
            'uniformExportDtS': float(uniform_dt_s),
            'uniformExportSamples': int(q_uniform.shape[0]),
            'ssMode': ss_mode,
            'minimaThresholdMode': str(ruckig_config.minima_threshold_mode),
            'minimaIdx': result.minima_idx.astype(int).tolist(),
            'minimaV': result.minima_v.astype(float).tolist(),
            'segmentPoints': [int(v) for v in result.segment_points],
            'segmentTargets': [float(v) for v in result.segment_targets],
            'segmentTargetVelocities': result.segment_target_velocities.astype(float).tolist(),
            'config': merged_config,
        }

        logs.append(f"duration={summary['duration']:.6f}s")
        logs.append(f"samples={summary['samples']}")
        logs.append(f"minima_count={len(summary['minimaIdx'])}")

        mvc_chart = _downsample_series(result.prepared.gridpoints, result.mvc_upper)
        profile_chart = _downsample_profile(result.ts, result.s_path, result.sd_path, result.sdd_path)

        return {
            'ok': True,
            'summary': summary,
            'trajectory': {
                'duration': float(result.ts[-1]),
                'ts': result.ts.astype(float).tolist(),
                'qs': result.qs.astype(float).tolist(),
                'qds': result.qds.astype(float).tolist(),
                'qdds': result.qdds.astype(float).tolist(),
                'tUniform': t_uniform.astype(float).tolist(),
                'qUniform': q_uniform.astype(float).tolist(),
            },
            'mvc': {
                'gridpoints': result.prepared.gridpoints.astype(float).tolist(),
                'mvcUpper': result.mvc_upper.astype(float).tolist(),
                'minimaIdx': result.minima_idx.astype(int).tolist(),
                'minimaV': result.minima_v.astype(float).tolist(),
                'chart': mvc_chart,
            },
            'pathProfile': {
                't': result.ts.astype(float).tolist(),
                's': result.s_path.astype(float).tolist(),
                'sd': result.sd_path.astype(float).tolist(),
                'sdd': result.sdd_path.astype(float).tolist(),
                'chart': profile_chart,
            },
            'waypoints': {
                'raw': result.prepared.waypoints_raw.astype(float).tolist(),
                'smooth': result.prepared.waypoints_smooth.astype(float).tolist(),
            },
            'warnings': warnings,
            'logs': logs,
        }
    finally:
        if tmp_path is not None and tmp_path.exists():
            tmp_path.unlink(missing_ok=True)


def main() -> None:
    try:
        raw = sys.stdin.buffer.read().decode('utf-8').strip()
        payload = json.loads(raw) if raw else {}
        action = str(payload.get('action') or 'run').strip().lower()

        if action == 'health':
            result = _health()
        elif action == 'run':
            result = _run(payload)
        else:
            raise ValueError(f'Unsupported action: {action}')

        sys.stdout.write(json.dumps(_json_safe(result), ensure_ascii=False, allow_nan=False))
    except Exception as error:
        error_payload = {
            'ok': False,
            'error': str(error),
            'traceback': traceback.format_exc(),
        }
        sys.stdout.write(json.dumps(_json_safe(error_payload), ensure_ascii=False, allow_nan=False))
        sys.exit(1)


if __name__ == '__main__':
    main()
