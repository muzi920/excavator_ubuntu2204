from pathlib import Path

import numpy as np


def list_pcd_files(source_dir, pattern):
    root = Path(source_dir)
    return sorted(root.glob(pattern))


def read_pcd_xyz(path):
    path = Path(path)
    with path.open("rb") as f:
        points_count = None
        fields = None
        sizes = None
        types = None

        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path} 在 DATA 字段前提前结束。")

            text = line.decode("ascii", "replace").strip()
            if text.startswith("FIELDS "):
                fields = text.split()[1:]
            elif text.startswith("SIZE "):
                sizes = [int(x) for x in text.split()[1:]]
            elif text.startswith("TYPE "):
                types = text.split()[1:]
            elif text.startswith("POINTS "):
                points_count = int(text.split()[1])
            elif text.startswith("DATA "):
                data_mode = text.split()[1].lower()
                if data_mode != "binary":
                    raise ValueError(f"{path} 仅支持 binary PCD，当前是 {data_mode}。")
                break

        if fields != ["x", "y", "z"]:
            raise ValueError(f"{path} 的 FIELDS 不是 x y z: {fields}")
        if sizes != [4, 4, 4] or types != ["F", "F", "F"]:
            raise ValueError(f"{path} 的点格式不是 float32 xyz。")
        if points_count is None:
            raise ValueError(f"{path} 缺少 POINTS 字段。")

        raw = f.read(points_count * 12)
        if len(raw) != points_count * 12:
            raise ValueError(f"{path} 二进制长度不足，期望 {points_count * 12} 字节，实际 {len(raw)}。")

    points = np.frombuffer(raw, dtype=np.float32).reshape(points_count, 3).copy()
    finite_mask = np.isfinite(points).all(axis=1)
    return points[finite_mask]


def write_pcd_xyz(path, points):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points 必须是 Nx3。")

    points = points[np.isfinite(points).all(axis=1)]
    points_count = int(points.shape[0])

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {points_count}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {points_count}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")

    with path.open("wb") as f:
        f.write(header)
        f.write(points.tobytes(order="C"))


def write_pcd_xyzrgb(path, xyz, rgb_uint8):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    xyz = np.asarray(xyz, dtype=np.float32)
    rgb_uint8 = np.asarray(rgb_uint8, dtype=np.uint8)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz 必须是 Nx3。")
    if rgb_uint8.ndim != 2 or rgb_uint8.shape[1] != 3:
        raise ValueError("rgb_uint8 必须是 Nx3。")
    if xyz.shape[0] != rgb_uint8.shape[0]:
        raise ValueError("xyz 和 rgb_uint8 行数必须相同。")

    finite_mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite_mask]
    rgb_uint8 = rgb_uint8[finite_mask]
    points_count = int(xyz.shape[0])

    rgb_u32 = (
        rgb_uint8[:, 0].astype(np.uint32) << np.uint32(16)
        | rgb_uint8[:, 1].astype(np.uint32) << np.uint32(8)
        | rgb_uint8[:, 2].astype(np.uint32)
    )
    rgb_f32 = rgb_u32.view(np.float32)
    out = np.concatenate([xyz.astype(np.float32, copy=False), rgb_f32.reshape(-1, 1)], axis=1)

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z rgb",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            f"WIDTH {points_count}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {points_count}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")

    with path.open("wb") as f:
        f.write(header)
        f.write(out.astype(np.float32, copy=False).tobytes(order="C"))
