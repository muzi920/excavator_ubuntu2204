import math


def _axis_samples(length, step):
    length = float(length)
    step = max(float(step), 1e-6)
    half = length * 0.5
    count = max(1, int(round(length / step))) + 1
    if count <= 1:
        return [0.0]
    real_step = length / float(count - 1)
    return [(-half + i * real_step) for i in range(count)]


def project_point_to_local(rect, x, y):
    center = rect["center"]
    cx = float(center["x"])
    cy = float(center["y"])
    yaw_deg = float(rect.get("yaw_deg", 0.0))
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    dx = float(x) - cx
    dy = float(y) - cy
    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw
    return float(local_x), float(local_y)


def point_in_rect(rect, x, y):
    local_x, local_y = project_point_to_local(rect, x, y)
    inside = abs(local_x) <= (float(rect["length"]) * 0.5) and abs(local_y) <= (float(rect["width"]) * 0.5)
    return inside, local_x, local_y


def order_local_points(points, pattern, row_step):
    row_step = max(float(row_step), 1e-6)
    rows = {}
    for point in points:
        row_key = int(round(float(point["local_y"]) / row_step))
        rows.setdefault(row_key, []).append(point)

    ordered = []
    for row_idx, row_key in enumerate(sorted(rows.keys())):
        row_points = rows[row_key]
        row_points.sort(key=lambda item: float(item["local_x"]), reverse=(pattern == "boustrophedon" and row_idx % 2 == 1))
        for col_index, point in enumerate(row_points):
            point["row_index"] = row_idx
            point["col_index"] = col_index
            ordered.append(point)
    return ordered


def sample_rect_points(rect, sampling):
    center = rect["center"]
    cx = float(center["x"])
    cy = float(center["y"])
    cz = float(center.get("z", 0.0))
    length = float(rect["length"])
    width = float(rect["width"])
    yaw_deg = float(rect.get("yaw_deg", 0.0))
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    step_length = float(sampling.get("step_along_length", 0.12))
    step_width = float(sampling.get("step_along_width", 0.15))
    pattern = str(sampling.get("pattern", "boustrophedon")).lower()

    local_xs = _axis_samples(length, step_length)
    local_ys = _axis_samples(width, step_width)

    points = []
    sample_index = 1
    for row_index, local_y in enumerate(local_ys):
        row_xs = list(local_xs)
        if pattern == "boustrophedon" and row_index % 2 == 1:
            row_xs.reverse()

        for col_index, local_x in enumerate(row_xs):
            x = cx + local_x * cos_yaw - local_y * sin_yaw
            y = cy + local_x * sin_yaw + local_y * cos_yaw
            points.append(
                {
                    "sample_index": sample_index,
                    "row_index": row_index,
                    "col_index": col_index,
                    "local_x": float(local_x),
                    "local_y": float(local_y),
                    "x": float(x),
                    "y": float(y),
                    "z": cz,
                }
            )
            sample_index += 1

    return points
