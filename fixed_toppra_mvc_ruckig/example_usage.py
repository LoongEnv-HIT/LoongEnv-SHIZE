from pathlib import Path
import sys

import numpy as np
import roboticstoolbox as rtb

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fixed_toppra_mvc_ruckig import (
    PathPreparationConfig,
    RuckigConfig,
    ToppraConfig,
    run_mvc_ruckig_pipeline,
)


def main() -> None:
    robot = rtb.models.DH.hsr650()
    vlim = np.tile(np.array([[-1.0, 1.0]]), (6, 1))
    alim = np.tile(np.array([[-10.0, 10.0]]), (6, 1))

    result = run_mvc_ruckig_pipeline(
        robot_model=robot,
        vlim=vlim,
        alim=alim,
        path_config=PathPreparationConfig(
            input_path="traj_seg_001.txt",
            resample_points=1000,
            joint_smooth_passes=60,
            joint_smooth_blend=0.49,
            swap_xy_for_ee=True,
        ),
        toppra_config=ToppraConfig(
            du=1e-3,
            dx=5e-2,
            dc=9e-3,
            interpolation_scheme=1,
            solver_wrapper="ecos",
        ),
        ruckig_config=RuckigConfig(
            dt=0.001,
            v_max=0.02,
            a_max=0.02,
            j_max=0.04,
            intermediate_velocity=0.005,
            minima_count=None,
            minima_gap=12,
            minima_threshold_mode="mean",
            minima_max_value=None,
            minima_smooth_window=21,
            junction_velocity_scale=0.8,
        ),
    )

    np.savetxt("fixed_toppra_mvc_ruckig_result.txt", result.qs, fmt="%.9f")
    print("minima_idx =", result.minima_idx.tolist())
    print("segment_points =", result.segment_points)
    print("duration =", float(result.ts[-1]))
    print("qs_shape =", result.qs.shape)


if __name__ == "__main__":
    main()
