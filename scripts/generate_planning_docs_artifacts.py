"""
Generate planning demo artifacts for repository docs.

Outputs:
- docs/planning_demo_summary.json
- docs/images/planning_mvc.svg
- docs/images/planning_profile_s.svg
- docs/images/planning_profile_sd.svg
- docs/images/planning_joint_uniform.svg
"""

from __future__ import annotations

import json
import math
import importlib.util
from pathlib import Path


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


def main() -> None:
    repo = Path(__file__).resolve().parent.parent
    run_bridge = load_bridge_run(repo)
    images_dir = repo / "docs" / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    sample_text = (repo / "public" / "samples" / "er15_waypoints_sample.csv").read_text(encoding="utf-8")

    payload = {
        "waypointText": sample_text,
        "fileName": "er15_waypoints_sample.csv",
        "limits": {
            "velocity": [1, 1, 1, 1, 1, 1],
            "acceleration": [10, 10, 10, 10, 10, 10],
        },
        "config": {
            "resample_points": 1000,
            "smooth_passes": 40,
            "smooth_blend": 0.49,
            "dt": 0.002,
            "v_max": 0.02,
            "a_max": 0.02,
            "j_max": 0.04,
            "minima_count": None,
            "minima_gap": 12,
            "minima_threshold_mode": "mean",
            "minima_max_value": None,
            "junction_velocity_scale": 0.8,
        },
        "ssMode": "arc",
        "playbackDtMs": 4,
    }

    result = run_bridge(payload)

    compact = {
        "summary": result["summary"],
        "algorithm": {
            "name": "fixed_toppra_mvc_ruckig",
            "pipeline": "TOPPRA + MVC + segmented Ruckig",
        },
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

    print("Generated docs demo artifacts successfully.")


if __name__ == "__main__":
    main()
