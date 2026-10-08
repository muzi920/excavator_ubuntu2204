"""Geometry helpers for point-cloud detections."""

import numpy as np


def highest_point_in_oriented_box(points, box, tolerance=1e-5):
    """
    Return the point with the greatest z inside an OpenPCDet 3D box.

    ``points`` must contain at least x, y and z columns. ``box`` uses the
    OpenPCDet layout ``[cx, cy, cz, dx, dy, dz, yaw]``. Height is measured
    along the positive z axis of the point-cloud frame.
    """
    points_array = np.asarray(points)
    box_array = np.asarray(box)

    if points_array.ndim != 2 or points_array.shape[1] < 3:
        raise ValueError('points must have shape (N, C), where C >= 3')
    if box_array.shape != (7,):
        raise ValueError('box must have shape (7,)')
    if points_array.shape[0] == 0:
        return None

    xyz = points_array[:, :3]
    relative = xyz - box_array[:3]
    cosine = np.cos(box_array[6])
    sine = np.sin(box_array[6])

    # Rotate global-frame offsets by -yaw into the box-local frame.
    local_x = relative[:, 0] * cosine + relative[:, 1] * sine
    local_y = -relative[:, 0] * sine + relative[:, 1] * cosine
    half_size = np.abs(box_array[3:6]) * 0.5 + tolerance

    inside = (
        (np.abs(local_x) <= half_size[0])
        & (np.abs(local_y) <= half_size[1])
        & (np.abs(relative[:, 2]) <= half_size[2])
    )
    indices = np.flatnonzero(inside)
    if indices.size == 0:
        return None

    highest_index = indices[np.argmax(xyz[indices, 2])]
    return np.array(xyz[highest_index], dtype=np.float32, copy=True)
