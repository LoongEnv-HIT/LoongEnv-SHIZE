# Fixed TOPPRA MVC Ruckig

This folder contains a standalone, reusable implementation of the current joint-path pipeline:

1. Smooth the input joint waypoints
2. Build a TOPPRA path parameterization problem
3. Compute the MVC upper envelope from `sqrt(X[:, 1])`
4. Detect local minima on the MVC upper envelope
5. Keep minima below the mean of the MVC upper envelope by default
6. Run segmented 1-DoF Ruckig on the path parameter `s`
7. Stitch segments and map back to joint space

It also includes the current Ruckig fixes:

- `min_velocity = 0.0` so the path parameter never runs backward
- `min_acceleration = -a_max` so the solver can still decelerate
- segment target speed is reduced by `junction_velocity_scale`
- short segments clamp target speed to a conservative feasible bound
- segment retries automatically fall back through softer target speeds
- final `sd/sdd` and final joint `qd/qdd` are explicitly set to zero

## Files

- `pipeline.py`: full reusable implementation
- `core.py`: slim API for direct use
- `cli.py`: command-line entrypoint
- `example_usage.py`: minimal Python example
- `requirements.txt`: dependency list
- `__init__.py`, `__main__.py`: package exports and `python -m` entry

## Python API

```python
import numpy as np
import roboticstoolbox as rtb

from fixed_toppra_mvc_ruckig import CorePipelineConfig, run_core_pipeline

robot = rtb.models.DH.hsr650()
vlim = np.tile(np.array([[-1.0, 1.0]]), (6, 1))
alim = np.tile(np.array([[-10.0, 10.0]]), (6, 1))

result = run_core_pipeline(
    robot_model=robot,
    waypoints_or_path="traj_seg_001.txt",
    vlim=vlim,
    alim=alim,
    config=CorePipelineConfig(
        resample_points=1000,
        smooth_passes=60,
        smooth_blend=0.49,
        dt=0.001,
        v_max=0.02,
        a_max=0.02,
        j_max=0.04,
        minima_count=None,
        minima_gap=12,
        minima_threshold_mode="mean",
        minima_max_value=None,
        junction_velocity_scale=0.8,
    ),
)

print(result.minima_idx)
print(result.segment_target_velocities)
print(result.qs.shape)
```

## CLI

Run from the workspace root:

```bash
pip install -e fixed_toppra_mvc_ruckig

python -m fixed_toppra_mvc_ruckig \
  --input traj_seg_001.txt \
  --robot hsr650 \
  --vmax-joints 1 1 1 1 1 1 \
  --amax-joints 10 10 10 10 10 10 \
  --output-dir outputs/fixed_toppra_mvc_ruckig
```

After editable install, you can also use the console script:

```bash
fixed-toppra-mvc-ruckig \
  --input traj_seg_001.txt \
  --robot hsr650 \
  --vmax-joints 1 1 1 1 1 1 \
  --amax-joints 10 10 10 10 10 10 \
  --output-dir outputs/fixed_toppra_mvc_ruckig
```

Without installation, it still works directly from the source tree:

```bash
python -m fixed_toppra_mvc_ruckig \
  --input traj_seg_001.txt \
  --robot hsr650 \
  --vmax-joints 1 1 1 1 1 1 \
  --amax-joints 10 10 10 10 10 10 \
  --output-dir outputs/fixed_toppra_mvc_ruckig
```

Important defaults:

- `v_max = 0.02`
- `a_max = 0.02`
- `j_max = 0.04`
- `minima_threshold_mode = mean`
- `junction_velocity_scale = 0.8`

Optional minima filtering modes:

- `--minima-threshold-mode mean`: keep minima below `mean(mvc_upper)`
- `--minima-threshold-mode fixed --minima-max-value 0.02`: fixed threshold
- `--minima-threshold-mode none`: keep all valid minima

## Outputs

The CLI writes:

- `joint_position.txt`
- `joint_velocity.txt`
- `joint_acceleration.txt`
- `joint_position_uniform.txt`
- `mvc_upper.txt`
- `path_profile.csv`
- `summary.json`

`summary.json` includes:

- `minima_idx`
- `minima_v`
- `segment_targets`
- `segment_target_velocities`
- `minima_threshold_mode`
- `minima_threshold_value`

## Notes

- `swap_xy_for_ee=True` is available if your project uses the XY-swapped FK convention.
- The pipeline assumes a monotonic path parameter `s in [0, 1]`.
- End-state velocity and acceleration are clamped to zero in the exported result.
