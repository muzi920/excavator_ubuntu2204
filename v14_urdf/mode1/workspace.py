import math


def radius_yaw_deg(x, y):
    radius = math.sqrt(float(x) * float(x) + float(y) * float(y))
    yaw_deg = math.degrees(math.atan2(float(y), float(x)))
    return radius, yaw_deg


def within_workspace(x, y, z, workspace):
    if not workspace:
        radius, yaw_deg = radius_yaw_deg(x, y)
        return True, None, radius, yaw_deg

    radius, yaw_deg = radius_yaw_deg(x, y)
    if radius < float(workspace["r_min"]) or radius > float(workspace["r_max"]):
        return False, "radius", radius, yaw_deg
    if yaw_deg < float(workspace["yaw_min_deg"]) or yaw_deg > float(workspace["yaw_max_deg"]):
        return False, "yaw", radius, yaw_deg
    if z < float(workspace["z_min"]) or z > float(workspace["z_max"]):
        return False, "z", radius, yaw_deg
    return True, None, radius, yaw_deg


def build_constraints_mask(constraints_data):
    if not constraints_data:
        return None
    zslice_bounds = constraints_data.get("bucket_tip_zslice_bounds")
    if not isinstance(zslice_bounds, dict):
        return None
    slice_bin_points = zslice_bounds.get("slice_bin_points")
    if not isinstance(slice_bin_points, list) or not slice_bin_points:
        return None

    xy_bin = float(zslice_bounds["xy_bin"])
    valid_bins = {
        (int(round(float(item["x"]) / xy_bin)), int(round(float(item["y"]) / xy_bin)))
        for item in slice_bin_points
    }
    return {
        "xy_bin": xy_bin,
        "valid_bins": valid_bins,
        "min_x": float(zslice_bounds["min_x"]),
        "max_x": float(zslice_bounds["max_x"]),
        "min_y": float(zslice_bounds["min_y"]),
        "max_y": float(zslice_bounds["max_y"]),
    }


def point_in_constraints_mask(x, y, mask):
    if not mask:
        return True, None, None, None

    if x < mask["min_x"] or x > mask["max_x"] or y < mask["min_y"] or y > mask["max_y"]:
        return False, "bounds", None, None

    xy_bin = mask["xy_bin"]
    bx = int(round(float(x) / xy_bin))
    by = int(round(float(y) / xy_bin))
    if (bx, by) not in mask["valid_bins"]:
        return False, "mask", float(bx) * xy_bin, float(by) * xy_bin

    return True, None, float(bx) * xy_bin, float(by) * xy_bin
