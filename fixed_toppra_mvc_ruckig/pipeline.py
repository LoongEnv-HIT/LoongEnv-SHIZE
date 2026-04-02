from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint


@dataclass
class PathPreparationConfig:
    input_path: str = "traj_seg_001.txt"
    grid_N: int | None = None
    ss_mode: str = "arc"
    eps: float = 1e-12
    dtype: type = np.float64
    resample_points: int = 1000
    joint_smooth_passes: int = 60
    joint_smooth_blend: float = 0.49
    arc_smoothing_alpha: float | None = None
    swap_xy_for_ee: bool = False


@dataclass
class ToppraConfig:
    du: float = 1e-3
    dx: float = 5e-2
    dc: float = 9e-3
    interpolation_scheme: int = 1
    solver_wrapper: str = "ecos"


@dataclass
class RuckigConfig:
    dt: float = 0.001
    v_max: float = 0.02
    a_max: float = 0.02
    j_max: float = 0.04
    intermediate_velocity: float = 0.005
    minima_count: int | None = None
    minima_gap: int = 12
    minima_threshold_mode: str = "mean"
    minima_max_value: float | None = None
    minima_smooth_window: int = 21
    junction_velocity_scale: float = 0.8
    max_steps: int = 200000


@dataclass
class PreparedPathResult:
    gridpoints: np.ndarray
    ss: np.ndarray
    waypoints_raw: np.ndarray
    waypoints_smooth: np.ndarray
    ee_points: np.ndarray
    path: ta.SplineInterpolator


@dataclass
class PipelineResult:
    prepared: PreparedPathResult
    toppra_instance: algo.TOPPRA
    X: np.ndarray
    K: np.ndarray
    sd_vec: np.ndarray | None
    mvc_upper: np.ndarray
    minima_idx: np.ndarray
    segment_points: list[int]
    segment_targets: list[float]
    segment_target_velocities: np.ndarray
    ts: np.ndarray
    s_path: np.ndarray
    sd_path: np.ndarray
    sdd_path: np.ndarray
    qs: np.ndarray
    qds: np.ndarray
    qdds: np.ndarray
    minima_s: np.ndarray
    minima_v: np.ndarray


def _ema_smooth(data: np.ndarray, alpha: float) -> np.ndarray:
    alpha = float(np.clip(alpha, 1e-6, 1.0))
    out = np.array(data, dtype=float, copy=True)
    for i in range(1, out.shape[0]):
        out[i] = (1.0 - alpha) * out[i - 1] + alpha * out[i]
    return out


def _map_toolbox_xyz(xyz_toolbox: np.ndarray, swap_xy_for_ee: bool) -> np.ndarray:
    xyz_toolbox = np.asarray(xyz_toolbox, dtype=float).reshape(3)
    if swap_xy_for_ee:
        return np.array([xyz_toolbox[1], xyz_toolbox[0], xyz_toolbox[2]], dtype=float)
    return xyz_toolbox


def _candidate_dep_dirs() -> list[Path]:
    here = Path(__file__).resolve()
    return [
        here.parent / ".deps",
        here.parent.parent / ".deps",
        Path.cwd() / ".deps",
    ]


def load_joint_waypoints(csv_path: str, dtype=np.float64) -> np.ndarray:
    csv_path = str(csv_path)
    required = ["J1", "J2", "J3", "J4", "J5", "J6"]
    with open(csv_path, "r", encoding="utf-8-sig") as f:
        first_line = f.readline().strip()

    is_csv_header = ("," in first_line) or ("J1" in first_line)
    suffix = Path(csv_path).suffix.lower()
    if is_csv_header or suffix == ".csv":
        df = pd.read_csv(csv_path)
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise ValueError(f"CSV is missing columns: {missing}")
        way_pts = df[required].to_numpy(dtype=dtype)
    else:
        way_pts = np.loadtxt(csv_path, dtype=dtype)
        way_pts = np.asarray(way_pts, dtype=dtype)
        if way_pts.ndim == 1:
            way_pts = way_pts.reshape(1, -1)
        if way_pts.shape[1] != 6:
            raise ValueError(f"Expected 6 joint columns, got {way_pts.shape[1]} from {csv_path}")
    return way_pts


def resample_waypoints(way_pts: np.ndarray, n_points: int = 1000) -> np.ndarray:
    w = np.asarray(way_pts, dtype=float)
    n_src = w.shape[0]
    if n_src <= 1 or n_points <= 1:
        return w.copy()
    s_src = np.linspace(0.0, 1.0, n_src)
    s_dst = np.linspace(0.0, 1.0, int(n_points))
    out = np.zeros((len(s_dst), w.shape[1]), dtype=float)
    for j in range(w.shape[1]):
        out[:, j] = np.interp(s_dst, s_src, w[:, j])
    return out


def smooth_waypoints_along_arc(way_pts: np.ndarray, alpha: float = 0.12) -> np.ndarray:
    w = np.asarray(way_pts, dtype=float)
    if w.shape[0] < 3:
        return w.copy()
    dp = np.diff(w, axis=0)
    ds = np.linalg.norm(dp, axis=1)
    s = np.hstack(([0.0], np.cumsum(ds)))
    if s[-1] <= 1e-12:
        return w.copy()

    sn = s / s[-1]
    sn_soft = _ema_smooth(sn.reshape(-1, 1), alpha).reshape(-1)
    sn_soft[0] = 0.0
    sn_soft[-1] = 1.0
    sn_soft = np.maximum.accumulate(sn_soft)
    sn_soft = np.clip(sn_soft, 0.0, 1.0)

    w_soft = np.zeros_like(w)
    for j in range(w.shape[1]):
        w_soft[:, j] = np.interp(sn_soft, sn, w[:, j])
    return w_soft


def smooth_waypoints_joint_laplacian(
    way_pts: np.ndarray, passes: int = 2, blend: float = 0.2
) -> np.ndarray:
    w = np.asarray(way_pts, dtype=float)
    if w.shape[0] < 3:
        return w.copy()

    ws = np.unwrap(w, axis=0)
    blend = float(np.clip(blend, 0.0, 0.49))
    passes = max(int(passes), 0)
    for _ in range(passes):
        nxt = ws.copy()
        nxt[1:-1, :] = (
            (1.0 - 2.0 * blend) * ws[1:-1, :]
            + blend * ws[:-2, :]
            + blend * ws[2:, :]
        )
        nxt[0, :] = ws[0, :]
        nxt[-1, :] = ws[-1, :]
        ws = nxt
    return ws


def compute_ee_arc_parameter(
    robot_model,
    way_pts: np.ndarray,
    dtype=np.float64,
    swap_xy_for_ee: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray(way_pts, dtype=dtype)
    n = q.shape[0]
    p_ee = np.zeros((n, 3), dtype=dtype)
    for i in range(n):
        Ti = robot_model.fkine(q[i, :])
        p_ee[i, :] = _map_toolbox_xyz(np.asarray(Ti.t, dtype=dtype).reshape(3), swap_xy_for_ee)

    delta_s = np.linalg.norm(np.diff(p_ee, axis=0), axis=1)
    s = np.hstack(([0.0], np.cumsum(delta_s)))
    total_s = max(float(s[-1]), 1e-12)
    ss = s / total_s
    return ss, delta_s, p_ee


def prepare_path(
    robot_model,
    config: PathPreparationConfig | None = None,
) -> PreparedPathResult:
    if config is None:
        config = PathPreparationConfig()

    dtype = config.dtype
    way_pts = load_joint_waypoints(config.input_path, dtype=dtype)
    way_pts_raw = resample_waypoints(way_pts, n_points=config.resample_points)
    way_pts_smooth = smooth_waypoints_joint_laplacian(
        way_pts_raw,
        passes=config.joint_smooth_passes,
        blend=config.joint_smooth_blend,
    )
    if config.arc_smoothing_alpha is not None:
        way_pts_smooth = smooth_waypoints_along_arc(
            way_pts_smooth, alpha=float(config.arc_smoothing_alpha)
        )

    if config.ss_mode not in {"time", "arc"}:
        raise ValueError("ss_mode must be 'time' or 'arc'")

    if config.ss_mode == "time":
        ss = np.linspace(0.0, 1.0, way_pts_smooth.shape[0], dtype=dtype)
        ee_points = np.zeros((way_pts_smooth.shape[0], 3), dtype=dtype)
    else:
        ss, _, ee_points = compute_ee_arc_parameter(
            robot_model,
            way_pts_smooth,
            dtype=dtype,
            swap_xy_for_ee=config.swap_xy_for_ee,
        )

    ss = np.asarray(ss, dtype=dtype)
    ss[0] = 0.0
    ss[-1] = 1.0
    ss = np.clip(ss, 0.0, 1.0)

    dss = np.diff(ss)
    if np.any(dss <= 0):
        ss_fix = ss.copy()
        for i in range(1, len(ss_fix)):
            if ss_fix[i] <= ss_fix[i - 1]:
                ss_fix[i] = ss_fix[i - 1] + config.eps
        ss_fix = (ss_fix - ss_fix[0]) / max(float(ss_fix[-1] - ss_fix[0]), 1e-12)
        ss_fix[0] = 0.0
        ss_fix[-1] = 1.0
        ss = ss_fix

    if config.grid_N is None:
        gridpoints = ss.copy()
    else:
        gridpoints = np.linspace(0.0, 1.0, int(config.grid_N), dtype=dtype)
    if not (np.all(np.isfinite(gridpoints)) and np.all(np.diff(gridpoints) > 0)):
        raise ValueError("gridpoints are invalid: expected finite and strictly increasing.")

    path = ta.SplineInterpolator(ss, way_pts_smooth)
    return PreparedPathResult(
        gridpoints=np.asarray(gridpoints, dtype=float),
        ss=np.asarray(ss, dtype=float),
        waypoints_raw=np.asarray(way_pts_raw, dtype=float),
        waypoints_smooth=np.asarray(way_pts_smooth, dtype=float),
        ee_points=np.asarray(ee_points, dtype=float),
        path=path,
    )


def build_toppra_instance(
    path: ta.SplineInterpolator,
    gridpoints: np.ndarray,
    vlim: np.ndarray,
    alim: np.ndarray,
    config: ToppraConfig | None = None,
) -> algo.TOPPRA:
    if config is None:
        config = ToppraConfig()

    c_vel = constraint.JointVelocityConstraint(np.asarray(vlim, dtype=float))
    pc_acc = constraint.JointAccelerationConstraint(
        np.asarray(alim, dtype=float),
        discretization_scheme=constraint.DiscretizationType.Interpolation,
    )
    robust_pc_acc = constraint.RobustLinearConstraint(
        pc_acc,
        [config.du, config.dx, config.dc],
        config.interpolation_scheme,
    )
    return algo.TOPPRA(
        [c_vel, robust_pc_acc],
        path,
        gridpoints=np.asarray(gridpoints, dtype=float),
        solver_wrapper=config.solver_wrapper,
    )


def compute_mvc_profiles(instance: algo.TOPPRA) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray]:
    X = instance.compute_feasible_sets()
    K = instance.compute_controllable_sets(0, 0)
    _, sd_vec, _ = instance.compute_parameterization(0, 0)
    X = np.sqrt(X)
    K = np.sqrt(K)
    mvc_upper = np.asarray(X[:, 1], dtype=float)
    return X, K, sd_vec, mvc_upper


def detect_mvc_minima_indices(
    mvc_upper: np.ndarray,
    n_inner: int | None = None,
    min_gap: int = 12,
    max_value: float | None = None,
    smooth_window: int = 21,
) -> np.ndarray:
    y = np.asarray(mvc_upper, dtype=float).reshape(-1)
    n = len(y)
    if n < 3:
        return np.array([], dtype=int)
    keep_all = n_inner is None or int(n_inner) <= 0

    candidates = []
    for i in range(1, n - 1):
        if y[i] < y[i - 1] and y[i] <= y[i + 1]:
            candidates.append(i)

    if len(candidates) < (1 if keep_all else int(n_inner)):
        w = max(int(smooth_window), 3)
        if w % 2 == 0:
            w += 1
        if w >= n:
            w = max(3, n - 1 if n % 2 == 0 else n)
        if w % 2 == 0:
            w -= 1
        if w >= 3:
            kernel = np.ones(w, dtype=float) / float(w)
            y_pad = np.pad(y, (w // 2, w // 2), mode="edge")
            ys = np.convolve(y_pad, kernel, mode="valid")
        else:
            ys = y

        smooth_candidates = []
        for i in range(1, n - 1):
            if ys[i] <= ys[i - 1] and ys[i] <= ys[i + 1]:
                smooth_candidates.append(i)

        half_window = max(w // 2, 1)
        for i in smooth_candidates:
            lo = max(int(i) - half_window, 1)
            hi = min(int(i) + half_window + 1, n - 1)
            j = lo + int(np.argmin(y[lo:hi]))
            candidates.append(j)

    if not candidates:
        order = np.argsort(y)
        candidates = [int(i) for i in order if 0 < i < n - 1]

    candidates = np.array(sorted(set(candidates)), dtype=int)
    gap = max(int(min_gap), 1)
    clustered = []
    for i in candidates:
        i = int(i)
        if not clustered:
            clustered.append(i)
            continue
        if i - clustered[-1] < gap:
            if y[i] < y[clustered[-1]]:
                clustered[-1] = i
        else:
            clustered.append(i)

    if max_value is not None:
        clustered = [i for i in clustered if y[int(i)] < float(max_value)]
        if not clustered:
            return np.array([], dtype=int)

    if keep_all:
        return np.array(clustered, dtype=int)

    selected = []
    for idx in np.argsort(y[np.asarray(clustered, dtype=int)]):
        i = int(clustered[idx])
        if all(abs(i - j) >= gap for j in selected):
            selected.append(i)
        if len(selected) >= int(n_inner):
            break

    if len(selected) < int(n_inner):
        for i in np.argsort(y):
            i = int(i)
            if not (0 < i < n - 1):
                continue
            if all(abs(i - j) >= max(gap // 2, 1) for j in selected):
                selected.append(i)
            if len(selected) >= int(n_inner):
                break
    return np.array(sorted(selected[: int(n_inner)]), dtype=int)


def _resolve_minima_max_value(
    mvc_upper: np.ndarray,
    config: RuckigConfig,
) -> float | None:
    mode = str(config.minima_threshold_mode).strip().lower()
    if mode == "none":
        return None
    if mode == "fixed":
        return None if config.minima_max_value is None else float(config.minima_max_value)
    if mode == "mean":
        values = np.asarray(mvc_upper, dtype=float).reshape(-1)
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            return None
        return float(np.mean(finite))
    raise ValueError("minima_threshold_mode must be one of: 'none', 'fixed', 'mean'.")


def _segment_feasible_target_velocity(
    x_prev: float,
    x_target: float,
    v_prev: float,
    desired_v: float,
    v_max: float,
    a_max: float,
    j_max: float,
    safety: float = 0.98,
) -> float:
    seg_len = max(float(x_target) - float(x_prev), 0.0)
    if seg_len <= 0.0:
        return 0.0

    v_prev_nonneg = max(float(v_prev), 0.0)
    v_max_eff = max(float(v_max), 1e-9)
    a_max_eff = max(float(a_max), 1e-9)
    j_max_eff = max(float(j_max), 1e-9)
    desired_v = float(np.clip(desired_v, 0.0, v_max_eff))

    # Conservative distance-based caps to avoid infeasible short segments.
    acc_cap = np.sqrt(max(v_prev_nonneg * v_prev_nonneg + 2.0 * a_max_eff * seg_len, 0.0))
    if desired_v >= v_prev_nonneg:
        jerk_cap = (seg_len * np.sqrt(j_max_eff)) ** (2.0 / 3.0)
        feasible_cap = min(v_max_eff, acc_cap, jerk_cap)
    else:
        feasible_cap = min(v_max_eff, acc_cap)

    return float(np.clip(min(desired_v, safety * feasible_cap), 0.0, v_max_eff))


def _import_ruckig():
    try:
        from ruckig import InputParameter, OutputParameter, Result, Ruckig
        return InputParameter, OutputParameter, Result, Ruckig
    except Exception as exc:
        for deps_dir in _candidate_dep_dirs():
            if deps_dir.exists():
                sys.path.insert(0, str(deps_dir))
                try:
                    from ruckig import InputParameter, OutputParameter, Result, Ruckig
                    return InputParameter, OutputParameter, Result, Ruckig
                except Exception:
                    continue
        raise ImportError(
            "ruckig is not available. Install with: python -m pip install ruckig "
            "or python -m pip install --target .deps ruckig"
        ) from exc


def ruckig_1d(
    x0: float,
    v0: float,
    a0: float,
    x1: float,
    v1: float = 0.0,
    a1: float = 0.0,
    v_max: float = 0.02,
    a_max: float = 0.02,
    j_max: float = 0.04,
    dt: float = 0.004,
    max_steps: int = 200000,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    InputParameter, OutputParameter, Result, Ruckig = _import_ruckig()

    otg = Ruckig(1, float(dt))
    inp = InputParameter(1)
    out = OutputParameter(1)
    v_max_eff = max(float(v_max), 1e-9)
    a_max_eff = max(float(a_max), 1e-9)
    j_max_eff = max(float(j_max), 1e-9)

    inp.current_position = [float(x0)]
    inp.current_velocity = [float(np.clip(v0, 0.0, v_max_eff))]
    inp.current_acceleration = [float(a0)]
    inp.target_position = [float(x1)]
    inp.target_velocity = [float(np.clip(v1, 0.0, v_max_eff))]
    inp.target_acceleration = [float(a1)]
    inp.max_velocity = [v_max_eff]
    inp.min_velocity = [0.0]
    inp.max_acceleration = [a_max_eff]
    inp.min_acceleration = [-a_max_eff]
    inp.max_jerk = [j_max_eff]

    t = 0.0
    ts = [t]
    xs = [float(x0)]
    vs = [float(np.clip(v0, 0.0, v_max_eff))]
    accs = [float(a0)]
    for _ in range(int(max_steps)):
        try:
            res = otg.update(inp, out)
        except Exception as exc:
            raise RuntimeError(f"Ruckig 1D failed during update: {exc}") from exc
        if res < 0:
            raise RuntimeError(f"Ruckig 1D failed with code {res}.")

        x_new = float(out.new_position[0])
        v_new = float(np.clip(out.new_velocity[0], 0.0, v_max_eff))
        a_new = float(out.new_acceleration[0])
        t += float(dt)
        ts.append(t)
        xs.append(x_new)
        vs.append(v_new)
        accs.append(a_new)
        out.pass_to_input(inp)
        inp.current_velocity = [v_new]
        inp.current_acceleration = [a_new]
        if res == Result.Finished:
            break
    else:
        raise RuntimeError("Ruckig 1D did not finish within max_steps.")
    return np.asarray(ts), np.asarray(xs), np.asarray(vs), np.asarray(accs)


def concat_ruckig_segments(
    segments: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t_all, x_all, v_all, a_all = [], [], [], []
    t_offset = 0.0
    for i, (t, x, v, a) in enumerate(segments):
        if i == 0:
            t_use, x_use, v_use, a_use = t, x, v, a
        else:
            t_use, x_use, v_use, a_use = t[1:], x[1:], v[1:], a[1:]
        t_all.append(t_use + t_offset)
        x_all.append(x_use)
        v_all.append(v_use)
        a_all.append(a_use)
        t_offset = float((t_use + t_offset)[-1])
    return (
        np.concatenate(t_all),
        np.concatenate(x_all),
        np.concatenate(v_all),
        np.concatenate(a_all),
    )


def plan_segmented_ruckig_from_mvc(
    gridpoints: np.ndarray,
    mvc_upper: np.ndarray,
    config: RuckigConfig | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[int], list[float], np.ndarray, np.ndarray]:
    if config is None:
        config = RuckigConfig()

    gridpoints = np.asarray(gridpoints, dtype=float).reshape(-1)
    effective_max_value = _resolve_minima_max_value(mvc_upper, config)
    minima_idx = detect_mvc_minima_indices(
        mvc_upper,
        n_inner=config.minima_count,
        min_gap=config.minima_gap,
        max_value=effective_max_value,
        smooth_window=config.minima_smooth_window,
    )
    segment_points = [int(i + 1) for i in minima_idx] + [len(gridpoints)]
    segment_targets = [float(gridpoints[i]) for i in minima_idx] + [1.0]
    minima_v = np.asarray(mvc_upper, dtype=float)[minima_idx] if len(minima_idx) else np.array([], dtype=float)
    velocity_scale = max(float(config.junction_velocity_scale), 0.0)

    segments = []
    segment_target_velocities: list[float] = []
    x_prev = 0.0
    v_prev = 0.0
    a_prev = 0.0
    for i, x_target in enumerate(segment_targets):
        v_prev_nonneg = max(float(v_prev), 0.0)
        if i == len(segment_targets) - 1:
            segment_v_max = max(float(config.v_max), v_prev_nonneg)
            v_target = 0.0
        else:
            desired_v = max(float(minima_v[i] * velocity_scale), 0.0)
            # Let each minima keep its own mapped target velocity instead of
            # flattening everything to the global v_max clamp.
            segment_v_max = max(float(config.v_max), desired_v, v_prev_nonneg)
            v_target = _segment_feasible_target_velocity(
                x_prev=x_prev,
                x_target=x_target,
                v_prev=v_prev_nonneg,
                desired_v=desired_v,
                v_max=segment_v_max,
                a_max=float(config.a_max),
                j_max=float(config.j_max),
            )

        last_error: Exception | None = None
        seg = None
        retry_candidates = [
            v_target,
            0.5 * (v_target + v_prev_nonneg),
            v_prev_nonneg,
            0.0,
        ]
        dedup_retry_candidates: list[float] = []
        for candidate in retry_candidates:
            candidate = float(np.clip(candidate, 0.0, segment_v_max))
            if all(abs(candidate - existing) > 1e-12 for existing in dedup_retry_candidates):
                dedup_retry_candidates.append(candidate)

        for retry_v_target in dedup_retry_candidates:
            try:
                seg = ruckig_1d(
                    x0=x_prev,
                    v0=v_prev,
                    a0=a_prev,
                    x1=x_target,
                    v1=retry_v_target,
                    a1=0.0,
                    v_max=segment_v_max,
                    a_max=config.a_max,
                    j_max=config.j_max,
                    dt=config.dt,
                    max_steps=config.max_steps,
                )
                v_target = retry_v_target
                break
            except RuntimeError as exc:
                last_error = exc
                continue
        if seg is None:
            raise RuntimeError("Segmented Ruckig failed without returning a trajectory.") from last_error

        segment_target_velocities.append(v_target)
        segments.append(seg)
        _, x_seg, v_seg, a_seg = seg
        x_prev = float(x_seg[-1])
        v_prev = max(float(v_seg[-1]), 0.0)
        a_prev = float(a_seg[-1])

    ts, s_path, sd_path, sdd_path = concat_ruckig_segments(segments)
    s_path = np.clip(s_path, 0.0, 1.0)
    return (
        ts,
        s_path,
        sd_path,
        sdd_path,
        segment_points,
        segment_targets,
        np.asarray(segment_target_velocities, dtype=float),
        minima_idx,
    )


def map_path_to_joint_trajectory(
    path: ta.SplineInterpolator,
    ts: np.ndarray,
    ss: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    qs = np.asarray(path(ss), dtype=float)
    qds = np.gradient(qs, axis=0, edge_order=2) / np.gradient(ts)[:, None]
    qdds = np.gradient(qds, axis=0, edge_order=2) / np.gradient(ts)[:, None]
    if qds.shape[0] > 0:
        qds[-1, :] = 0.0
        qdds[-1, :] = 0.0
    return qs, qds, qdds


def resample_traj_uniform(ts: np.ndarray, qs: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    ts = np.asarray(ts, dtype=float).reshape(-1)
    qs = np.asarray(qs, dtype=float)
    t_end = float(ts[-1])
    t_uniform = np.arange(0.0, t_end + 0.5 * dt, dt)
    q_uniform = np.zeros((len(t_uniform), qs.shape[1]), dtype=float)
    for j in range(qs.shape[1]):
        q_uniform[:, j] = np.interp(t_uniform, ts, qs[:, j])
    return t_uniform, q_uniform


def run_mvc_ruckig_pipeline(
    robot_model,
    vlim: np.ndarray,
    alim: np.ndarray,
    path_config: PathPreparationConfig | None = None,
    toppra_config: ToppraConfig | None = None,
    ruckig_config: RuckigConfig | None = None,
) -> PipelineResult:
    prepared = prepare_path(robot_model, path_config)
    instance = build_toppra_instance(
        prepared.path,
        prepared.gridpoints,
        vlim=vlim,
        alim=alim,
        config=toppra_config,
    )
    X, K, sd_vec, mvc_upper = compute_mvc_profiles(instance)
    ts, s_path, sd_path, sdd_path, segment_points, segment_targets, segment_target_velocities, minima_idx = (
        plan_segmented_ruckig_from_mvc(
            prepared.gridpoints,
            mvc_upper,
            config=ruckig_config,
        )
    )
    if len(sd_path) > 0:
        sd_path[-1] = 0.0
        sdd_path[-1] = 0.0
    qs, qds, qdds = map_path_to_joint_trajectory(prepared.path, ts, s_path)
    minima_s = np.array(segment_targets[:-1], dtype=float)
    minima_v = np.interp(minima_s, prepared.gridpoints, mvc_upper) if len(minima_s) else np.array([], dtype=float)
    return PipelineResult(
        prepared=prepared,
        toppra_instance=instance,
        X=X,
        K=K,
        sd_vec=sd_vec,
        mvc_upper=mvc_upper,
        minima_idx=minima_idx,
        segment_points=segment_points,
        segment_targets=segment_targets,
        segment_target_velocities=segment_target_velocities,
        ts=ts,
        s_path=s_path,
        sd_path=sd_path,
        sdd_path=sdd_path,
        qs=qs,
        qds=qds,
        qdds=qdds,
        minima_s=minima_s,
        minima_v=minima_v,
    )
