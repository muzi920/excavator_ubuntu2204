import numpy as np


def filter_excavator_body(points, mask_config):
    r_center = float(mask_config.get("center_radius", 0.7))
    z_min = float(mask_config.get("z_min", 0.0))
    z_max = float(mask_config.get("z_max", 1.8))

    x_min = float(mask_config.get("box_x_min", -0.4))
    x_max = float(mask_config.get("box_x_max", 0.7))
    y_abs = float(mask_config.get("box_y_abs", 0.55))
    box_z_min = float(mask_config.get("box_z_min", 0.0))
    box_z_max = float(mask_config.get("box_z_max", 0.9))

    arm_enabled = bool(mask_config.get("arm_enabled", False))
    arm_x_min = float(mask_config.get("arm_x_min", 0.2))
    arm_x_max = float(mask_config.get("arm_x_max", 1.3))
    arm_y_abs = float(mask_config.get("arm_y_abs", 0.35))
    arm_z_min = float(mask_config.get("arm_z_min", 0.75))
    arm_z_max = float(mask_config.get("arm_z_max", 1.85))

    xy_sq = points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1]
    cylinder = (xy_sq <= (r_center * r_center)) & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)

    body_box = (
        (points[:, 0] >= x_min)
        & (points[:, 0] <= x_max)
        & (np.abs(points[:, 1]) <= y_abs)
        & (points[:, 2] >= box_z_min)
        & (points[:, 2] <= box_z_max)
    )

    if arm_enabled:
        arm_box = (
            (points[:, 0] >= arm_x_min)
            & (points[:, 0] <= arm_x_max)
            & (np.abs(points[:, 1]) <= arm_y_abs)
            & (points[:, 2] >= arm_z_min)
            & (points[:, 2] <= arm_z_max)
        )
    else:
        arm_box = np.zeros((len(points),), dtype=bool)

    remove_mask = cylinder | body_box | arm_box
    kept = points[~remove_mask]
    removed = points[remove_mask]
    stats = {
        "input_points": int(len(points)),
        "kept_points": int(len(kept)),
        "removed_points": int(len(removed)),
        "removed_ratio": float(len(removed) / max(1, len(points))),
        "cylinder_removed": int(cylinder.sum()),
        "body_box_removed": int(body_box.sum()),
        "arm_box_removed": int(arm_box.sum()),
        "arm_enabled": arm_enabled,
    }
    return kept, removed, stats
