"""
Generate planning demo artifacts for repository docs.

Outputs:
- docs/planning_demo_summary.json
- docs/images/planning_mvc.svg
- docs/images/planning_profile_s.svg
- docs/images/planning_profile_sd.svg
- docs/images/planning_joint_uniform.svg
- docs/images/planning_joint_validation.svg
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np

DEFAULT_DOC_CONFIG = {
    "resample_points": 1000,
    "smooth_passes": 60,
    "smooth_blend": 0.49,
    "dt": 0.001,
    "v_max": 0.02,
    "a_max": 0.02,
    "j_max": 0.04,
    "minima_count": None,
    "minima_gap": 12,
    "minima_threshold_mode": "mean",
    "minima_max_value": None,
    "junction_velocity_scale": 0.8,
}


def load_bridge_run(repo_root: Path):
    bridge_path = repo_root / "scripts" / "planning_bridge.py"
    spec = importlib.util.spec_from_file_location("planning_bridge_local", bridge_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load planning bridge at {bridge_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._run


def downsample(points, max_points=360):
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def clip_float(value: float, digits: int = 6) -> float:
    return float(f"{float(value):.{digits}f}")


def parse_input_waypoints(text: str, file_name: str) -> np.ndarray:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 2:
        raise RuntimeError("Input waypoint file must contain at least 2 non-empty lines.")

    lower_name = file_name.lower()
    csv_like = lower_name.endswith(".csv") or "," in lines[0]

    if csv_like:
        header = [item.strip() for item in lines[0].lstrip("\ufeff").split(",")]
        required = ["J1", "J2", "J3", "J4", "J5", "J6"]
        col = {name: idx for idx, name in enumerate(header)}
        if any(name not in col for name in required):
            raise RuntimeError("CSV header must contain J1..J6.")
        rows = []
        for raw in lines[1:]:
            parts = [item.strip() for item in raw.split(",")]
            rows.append([float(parts[col[name]]) for name in required])
        arr = np.asarray(rows, dtype=float)
    else:
        rows = []
        for raw in lines:
            parts = [item for item in raw.replace(",", " ").split() if item]
            if len(parts) != 6:
                raise RuntimeError("TXT row must contain exactly 6 numeric columns.")
            rows.append([float(item) for item in parts])
        arr = np.asarray(rows, dtype=float)

    if arr.shape[0] < 2 or arr.shape[1] != 6:
        raise RuntimeError("Parsed waypoints are invalid.")
    return arr


def validate_result(result: dict) -> dict:
    ts = np.asarray(result["trajectory"]["ts"], dtype=float)
    t_uniform = np.asarray(result["trajectory"]["tUniform"], dtype=float)
    q_uniform = np.asarray(result["trajectory"]["qUniform"], dtype=float)
    qs = np.asarray(result["trajectory"]["qs"], dtype=float)

    if ts.size < 2:
        raise RuntimeError("Invalid trajectory: ts has fewer than 2 samples.")
    if qs.shape[0] != ts.size:
        raise RuntimeError("Invalid trajectory: len(qs) != len(ts).")
    if t_uniform.size < 2:
        raise RuntimeError("Invalid trajectory: tUniform has fewer than 2 samples.")
    if q_uniform.shape[0] != t_uniform.size:
        raise RuntimeError("Invalid trajectory: len(qUniform) != len(tUniform).")
    if np.any(~np.isfinite(ts)) or np.any(~np.isfinite(t_uniform)) or np.any(~np.isfinite(q_uniform)):
        raise RuntimeError("Invalid trajectory: non-finite values detected.")

    dt = np.diff(ts)
    dt_u = np.diff(t_uniform)
    if np.any(dt <= 0):
        raise RuntimeError("Invalid trajectory: ts is not strictly increasing.")
    if np.any(dt_u <= 0):
        raise RuntimeError("Invalid trajectory: tUniform is not strictly increasing.")

    return {
        "tsStep": {
            "min": clip_float(dt.min()),
            "max": clip_float(dt.max()),
            "mean": clip_float(dt.mean()),
        },
        "uniformStep": {
            "min": clip_float(dt_u.min()),
            "max": clip_float(dt_u.max()),
            "mean": clip_float(dt_u.mean()),
        },
        "uniformJointRange": [
            {
                "joint": f"J{i + 1}",
                "min": clip_float(q_uniform[:, i].min()),
                "max": clip_float(q_uniform[:, i].max()),
            }
            for i in range(min(6, q_uniform.shape[1]))
        ],
    }


def to_polyline(points, x_key, y_key, width=1160, height=460, pad=48):
    pts = downsample(points)
    xs = [float(p[x_key]) for p in pts]
    ys = [float(p[y_key]) for p in pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if max_x - min_x < 1e-9:
        max_x += 1.0
    if max_y - min_y < 1e-9:
        max_y += 1.0

    def scale_x(x):
        return pad + (x - min_x) * (width - 2 * pad) / (max_x - min_x)

    def scale_y(y):
        return height - pad - (y - min_y) * (height - 2 * pad) / (max_y - min_y)

    poly = " ".join(f"{scale_x(x):.2f},{scale_y(y):.2f}" for x, y in zip(xs, ys))
    return poly, (min_x, max_x, min_y, max_y)


def write_single_series_svg(path, title, x_label, y_label, points, x_key, y_key, color="#0b7ad1"):
    width, height = 1160, 460
    pad = 48
    poly, (min_x, max_x, min_y, max_y) = to_polyline(points, x_key, y_key, width, height, pad)

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fbff"/>
  <text x="{pad}" y="26" font-family="Segoe UI, Arial" font-size="18" fill="#1f2937" font-weight="700">{title}</text>
  <line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#94a3b8" stroke-width="1"/>
  <line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#94a3b8" stroke-width="1"/>
  <polyline fill="none" stroke="{color}" stroke-width="2" points="{poly}"/>
  <text x="{width/2:.0f}" y="{height-10}" font-family="Segoe UI, Arial" font-size="12" fill="#475569">{x_label} [{min_x:.3f} .. {max_x:.3f}]</text>
  <text x="8" y="{height/2:.0f}" transform="rotate(-90 8 {height/2:.0f})" font-family="Segoe UI, Arial" font-size="12" fill="#475569">{y_label} [{min_y:.3f} .. {max_y:.3f}]</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def write_multi_joint_svg(path, title, t_values, q_values):
    width, height = 1160, 500
    pad = 56
    n = len(t_values)
    idx = downsample(list(range(n)), 380)
    ts = [float(t_values[i]) for i in idx]
    series = []
    for j in range(6):
        series.append([float(q_values[i][j]) for i in idx])

    min_x, max_x = min(ts), max(ts)
    min_y = min(min(s) for s in series)
    max_y = max(max(s) for s in series)

    if max_x - min_x < 1e-9:
        max_x += 1.0
    if max_y - min_y < 1e-9:
        max_y += 1.0

    def sx(x):
        return pad + (x - min_x) * (width - 2 * pad) / (max_x - min_x)

    def sy(y):
        return height - pad - (y - min_y) * (height - 2 * pad) / (max_y - min_y)

    colors = ["#0b7ad1", "#16a34a", "#7c3aed", "#d97706", "#dc2626", "#0ea5a8"]
    polylines = []
    legends = []
    for j, vals in enumerate(series):
        pts = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(ts, vals))
        polylines.append(f'<polyline fill="none" stroke="{colors[j]}" stroke-width="1.8" points="{pts}"/>')
        legends.append(
            f'<rect x="{780 + (j%3)*120}" y="{20 + (j//3)*20}" width="10" height="10" fill="{colors[j]}"/>'
            f'<text x="{796 + (j%3)*120}" y="{29 + (j//3)*20}" font-family="Segoe UI, Arial" font-size="12" fill="#334155">J{j+1}</text>'
        )

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fbff"/>
  <text x="{pad}" y="28" font-family="Segoe UI, Arial" font-size="18" fill="#1f2937" font-weight="700">{title}</text>
  <line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#94a3b8" stroke-width="1"/>
  <line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#94a3b8" stroke-width="1"/>
  {''.join(polylines)}
  {''.join(legends)}
  <text x="{width/2:.0f}" y="{height-12}" font-family="Segoe UI, Arial" font-size="12" fill="#475569">Time [s] ({min_x:.2f} .. {max_x:.2f})</text>
  <text x="10" y="{height/2:.0f}" transform="rotate(-90 10 {height/2:.0f})" font-family="Segoe UI, Arial" font-size="12" fill="#475569">Joint Position [rad] ({min_y:.2f} .. {max_y:.2f})</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def write_joint_validation_svg(path, title, q_uniform, waypoints_original):
    width, height = 1160, 820
    pad_left = 72
    pad_right = 24
    pad_top = 44
    pad_bottom = 24
    rows = 6
    panel_h = (height - pad_top - pad_bottom) / rows

    q_uniform = np.asarray(q_uniform, dtype=float)
    waypoints_original = np.asarray(waypoints_original, dtype=float)
    n_u = max(2, q_uniform.shape[0])
    n_w = max(2, waypoints_original.shape[0])

    u_full = np.linspace(0.0, 1.0, n_u)
    u_way = np.linspace(0.0, 1.0, n_w)
    keep_idx = downsample(list(range(n_u)), 420)
    colors = ["#0b7ad1", "#16a34a", "#7c3aed", "#d97706", "#dc2626", "#0ea5a8"]

    def sx(x):
        return pad_left + x * (width - pad_left - pad_right)

    panels = []
    for j in range(6):
        y0 = pad_top + j * panel_h
        y1 = y0 + panel_h - 8
        values_u = q_uniform[:, j]
        values_w = waypoints_original[:, j]
        y_min = float(min(values_u.min(), values_w.min()))
        y_max = float(max(values_u.max(), values_w.max()))
        if y_max - y_min < 1e-9:
            y_max += 1.0

        def sy(y):
            return y1 - (y - y_min) * (y1 - y0 - 14) / (y_max - y_min)

        pts_u = " ".join(
            f"{sx(u_full[i]):.2f},{sy(values_u[i]):.2f}" for i in keep_idx
        )
        dots_w = "".join(
            f'<circle cx="{sx(u_way[i]):.2f}" cy="{sy(values_w[i]):.2f}" r="2.0" fill="#0f172a" />'
            for i in range(n_w)
        )

        panels.append(
            f'''
  <line x1="{pad_left}" y1="{y1:.2f}" x2="{width - pad_right}" y2="{y1:.2f}" stroke="#e2e8f0" stroke-width="1"/>
  <line x1="{pad_left}" y1="{y0:.2f}" x2="{pad_left}" y2="{y1:.2f}" stroke="#e2e8f0" stroke-width="1"/>
  <polyline fill="none" stroke="{colors[j % len(colors)]}" stroke-width="1.8" points="{pts_u}"/>
  {dots_w}
  <text x="{pad_left - 10}" y="{(y0 + y1) / 2:.2f}" text-anchor="end" dominant-baseline="middle" font-family="Segoe UI, Arial" font-size="11" fill="#334155">J{j+1}</text>
  <text x="{width - pad_right}" y="{y0 + 11:.2f}" text-anchor="end" font-family="Segoe UI, Arial" font-size="10" fill="#64748b">[{y_min:.3f}, {y_max:.3f}]</text>
'''
        )

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fbff"/>
  <text x="{pad_left}" y="26" font-family="Segoe UI, Arial" font-size="18" fill="#1f2937" font-weight="700">{title}</text>
  <text x="{pad_left}" y="{height - 8}" font-family="Segoe UI, Arial" font-size="12" fill="#475569">Normalized progress u (0..1)</text>
  <text x="{width - 430}" y="26" font-family="Segoe UI, Arial" font-size="11" fill="#475569">line = qUniform, dots = original file waypoints</text>
  {''.join(panels)}
</svg>'''
    path.write_text(svg, encoding="utf-8")


def main() -> None:
    repo = Path(__file__).resolve().parent.parent
    run_bridge = load_bridge_run(repo)
    images_dir = repo / "docs" / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    sample_text = (repo / "public" / "samples" / "er15_waypoints_sample.csv").read_text(encoding="utf-8")
    input_waypoints = parse_input_waypoints(sample_text, "er15_waypoints_sample.csv")

    payload = {
        "waypointText": sample_text,
        "fileName": "er15_waypoints_sample.csv",
        "limits": {
            "velocity": [1, 1, 1, 1, 1, 1],
            "acceleration": [10, 10, 10, 10, 10, 10],
        },
        "config": {
            **DEFAULT_DOC_CONFIG,
        },
        "ssMode": "arc",
        "playbackDtMs": 4,
    }

    result = run_bridge(payload)
    diagnostics = validate_result(result)

    compact = {
        "summary": result["summary"],
        "algorithm": {
            "name": "fixed_toppra_mvc_ruckig",
            "pipeline": "TOPPRA + MVC + segmented Ruckig",
        },
        "input": {
            "sampleFile": "public/samples/er15_waypoints_sample.csv",
            "waypointCountOriginal": int(input_waypoints.shape[0]),
            "waypointCountPrepared": len(result["waypoints"]["raw"]),
            "config": DEFAULT_DOC_CONFIG,
        },
        "diagnostics": diagnostics,
        "chartPointCount": {
            "mvc": len(result["mvc"]["chart"]),
            "pathProfile": len(result["pathProfile"]["chart"]),
            "uniformSamples": len(result["trajectory"]["tUniform"]),
        },
    }
    (repo / "docs" / "planning_demo_summary.json").write_text(
        json.dumps(compact, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    write_single_series_svg(
        images_dir / "planning_mvc.svg",
        "MVC Upper Envelope (Demo Run)",
        "Path coordinate s",
        "Velocity upper bound",
        result["mvc"]["chart"],
        "x",
        "y",
    )

    write_single_series_svg(
        images_dir / "planning_profile_s.svg",
        "Path Profile s(t) (Demo Run)",
        "Time [s]",
        "Path coordinate s",
        result["pathProfile"]["chart"],
        "t",
        "s",
        color="#16a34a",
    )

    write_single_series_svg(
        images_dir / "planning_profile_sd.svg",
        "Path Speed sd(t) (Demo Run)",
        "Time [s]",
        "Path speed sd",
        result["pathProfile"]["chart"],
        "t",
        "sd",
        color="#dc2626",
    )

    write_multi_joint_svg(
        images_dir / "planning_joint_uniform.svg",
        "Uniform Playback Joint Trajectory qUniform(t) (Demo Run)",
        result["trajectory"]["tUniform"],
        result["trajectory"]["qUniform"],
    )

    write_joint_validation_svg(
        images_dir / "planning_joint_validation.svg",
        "Joint Validation: qUniform vs Input Waypoints (Demo Run)",
        result["trajectory"]["qUniform"],
        input_waypoints,
    )

    print("Generated docs demo artifacts successfully.")


if __name__ == "__main__":
    main()
