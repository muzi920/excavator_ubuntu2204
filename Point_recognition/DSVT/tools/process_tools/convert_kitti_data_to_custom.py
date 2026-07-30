#!/usr/bin/env python3
"""
Convert a local dataset from:

    kitti_data/
      kitti_labels/*.txt
      point_clouds/default/*.pcd

to the OpenPCDet-style CustomDataset layout used by this repository:

    data/custom/
      points/*.npy
      labels/*.txt
      ImageSets/train.txt
      ImageSets/val.txt

Notes
-----
This script supports KITTI-style label rows:

    cls trunc occl alpha bbox_left bbox_top bbox_right bbox_bottom
    h w l x y z ry

Without camera calibration files, it is impossible to convert true KITTI
camera-coordinate boxes into lidar coordinates. Therefore this script assumes
the trailing box fields (h, w, l, x, y, z, ry) are already expressed in the
point-cloud / lidar coordinate system, and only reorders them into the
CustomDataset format:

    x y z l w h angle name
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert PCD + KITTI-style labels to CustomDataset")
    parser.add_argument(
        "--src-root",
        type=Path,
        default=Path("kitti_data"),
        help="Source dataset root containing kitti_labels/ and point_clouds/default/",
    )
    parser.add_argument(
        "--dst-root",
        type=Path,
        default=Path("data/custom"),
        help="Destination CustomDataset root",
    )
    parser.add_argument(
        "--pcd-dir",
        type=Path,
        default=None,
        help="Optional override for the source PCD directory",
    )
    parser.add_argument(
        "--label-dir",
        type=Path,
        default=None,
        help="Optional override for the source label directory",
    )
    parser.add_argument(
        "--train-ratio",
        type=float,
        default=0.8,
        help="Train split ratio used when generating ImageSets",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for train/val split",
    )
    parser.add_argument(
        "--fake-intensity",
        type=float,
        default=0.0,
        help="Intensity value to append when the source PCD has no intensity field",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Only convert the first N samples, useful for quick debugging",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing .npy/.txt outputs",
    )
    parser.add_argument(
        "--assume-lidar-labels",
        action="store_true",
        help="Acknowledge that label box fields are already in lidar coordinates",
    )
    parser.add_argument(
        "--classes",
        nargs="*",
        default=None,
        help="Optional class whitelist, e.g. --classes Soil",
    )
    parser.add_argument(
        "--shuffle",
        action="store_true",
        help="Shuffle sample ids before splitting into train/val",
    )
    return parser.parse_args()


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def read_pcd(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Unexpected EOF while reading PCD header: {path}")
            decoded = line.decode("ascii", errors="strict").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                break
        data = f.read()

    header = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]

    fields = header.get("FIELDS")
    sizes = [int(x) for x in header.get("SIZE", [])]
    types = header.get("TYPE")
    counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
    width = int(header.get("WIDTH", ["0"])[0])
    height = int(header.get("HEIGHT", ["1"])[0])
    num_points = int(header.get("POINTS", [str(width * height)])[0])
    data_type = header.get("DATA", [""])[0].lower()

    if not fields:
        raise ValueError(f"Missing FIELDS in PCD header: {path}")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"Inconsistent PCD header layout: {path}")
    if data_type not in {"binary", "ascii"}:
        raise NotImplementedError(
            f"Unsupported PCD DATA type '{data_type}' in {path}. "
            "Only 'binary' and 'ascii' are supported."
        )

    if data_type == "ascii":
        text = data.decode("ascii", errors="strict")
        arr = np.loadtxt(text.splitlines(), dtype=np.float32)
        arr = np.atleast_2d(arr)
        return arr

    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        if typ == "F":
            base = {4: np.float32, 8: np.float64}.get(size)
        elif typ == "I":
            base = {1: np.int8, 2: np.int16, 4: np.int32, 8: np.int64}.get(size)
        elif typ == "U":
            base = {1: np.uint8, 2: np.uint16, 4: np.uint32, 8: np.uint64}.get(size)
        else:
            base = None
        if base is None:
            raise NotImplementedError(f"Unsupported PCD field type {typ}{size} in {path}")
        dtype_fields.append((name, base) if count == 1 else (name, base, (count,)))

    dtype = np.dtype(dtype_fields)
    arr = np.frombuffer(data, dtype=dtype, count=num_points)

    columns = []
    for name in fields:
        value = arr[name]
        if value.ndim == 1:
            columns.append(value.reshape(-1, 1))
        else:
            columns.append(value.reshape(len(arr), -1))
    return np.concatenate(columns, axis=1).astype(np.float32, copy=False)


def build_point_features(points: np.ndarray, fields: Sequence[str], fake_intensity: float) -> np.ndarray:
    field_to_index = {name: idx for idx, name in enumerate(fields)}
    required = ["x", "y", "z"]
    missing = [name for name in required if name not in field_to_index]
    if missing:
        raise ValueError(f"PCD is missing required fields {missing}")

    xyz = np.stack([points[:, field_to_index[name]] for name in required], axis=1).astype(np.float32)
    if "intensity" in field_to_index:
        intensity = points[:, field_to_index["intensity"]].reshape(-1, 1).astype(np.float32)
    else:
        intensity = np.full((xyz.shape[0], 1), fake_intensity, dtype=np.float32)
    return np.concatenate([xyz, intensity], axis=1)


def read_pcd_with_fields(path: Path, fake_intensity: float) -> np.ndarray:
    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Unexpected EOF while reading PCD header: {path}")
            decoded = line.decode("ascii", errors="strict").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                break

    fields = None
    for line in header_lines:
        if line.startswith("FIELDS "):
            fields = line.split()[1:]
            break
    if fields is None:
        raise ValueError(f"Missing FIELDS in PCD header: {path}")

    raw_points = read_pcd(path)
    return build_point_features(raw_points, fields, fake_intensity)


def parse_kitti_label_line(line: str) -> Tuple[str, np.ndarray]:
    parts = line.strip().split()
    if not parts:
        raise ValueError("Encountered empty label line")
    if len(parts) < 15:
        raise ValueError(
            "Expected KITTI-style label with at least 15 columns, "
            f"got {len(parts)} columns: {line.strip()}"
        )

    cls_name = parts[0]
    h, w, l, x, y, z, ry = map(float, parts[-7:])
    custom_box = np.array([x, y, z, l, w, h, ry], dtype=np.float32)
    return cls_name, custom_box


def convert_label_file(src_label: Path, dst_label: Path, class_whitelist: set[str] | None, overwrite: bool) -> List[str]:
    if dst_label.exists() and not overwrite:
        class_names = []
        for line in dst_label.read_text().splitlines():
            if not line.strip():
                continue
            class_names.append(line.split()[-1])
        return class_names

    class_names = []
    converted_lines = []
    for line in src_label.read_text().splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        cls_name, box = parse_kitti_label_line(stripped)
        if class_whitelist is not None and cls_name not in class_whitelist:
            continue
        if not np.all(np.isfinite(box)):
            raise ValueError(f"Non-finite box values found in {src_label}")
        class_names.append(cls_name)
        converted_lines.append(
            f"{box[0]:.6f} {box[1]:.6f} {box[2]:.6f} "
            f"{box[3]:.6f} {box[4]:.6f} {box[5]:.6f} {box[6]:.6f} {cls_name}"
        )

    dst_label.write_text("\n".join(converted_lines) + ("\n" if converted_lines else ""))
    return class_names


def write_split_file(path: Path, sample_ids: Iterable[str]) -> None:
    content = "\n".join(sample_ids)
    path.write_text(content + ("\n" if content else ""))


def split_sample_ids(
    sample_ids: List[str], train_ratio: float, shuffle: bool, seed: int
) -> Tuple[List[str], List[str]]:
    if not sample_ids:
        return [], []
    ids = list(sample_ids)
    if shuffle:
        rng = np.random.default_rng(seed)
        rng.shuffle(ids)

    if len(ids) == 1:
        return ids, ids

    train_count = int(math.floor(len(ids) * train_ratio))
    train_count = min(max(train_count, 1), len(ids) - 1)
    return ids[:train_count], ids[train_count:]


def collect_source_pairs(pcd_dir: Path, label_dir: Path) -> List[Tuple[str, Path, Path]]:
    sample_ids = sorted(p.stem for p in pcd_dir.glob("*.pcd"))
    pairs = []
    for sample_id in sample_ids:
        pcd_path = pcd_dir / f"{sample_id}.pcd"
        label_path = label_dir / f"{sample_id}.txt"
        if not label_path.exists():
            raise FileNotFoundError(f"Missing label file for sample {sample_id}: {label_path}")
        pairs.append((sample_id, pcd_path, label_path))
    if not pairs:
        raise FileNotFoundError(f"No .pcd files found in {pcd_dir}")
    return pairs


def main() -> None:
    args = parse_args()

    if not args.assume_lidar_labels:
        raise SystemExit(
            "Refusing to convert labels unless --assume-lidar-labels is provided.\n"
            "This script only reorders KITTI-style box fields and assumes they are "
            "already expressed in the lidar/point-cloud coordinate system."
        )

    if not (0.0 < args.train_ratio < 1.0):
        raise SystemExit("--train-ratio must be in the open interval (0, 1)")

    pcd_dir = args.pcd_dir if args.pcd_dir is not None else args.src_root / "point_clouds" / "default"
    label_dir = args.label_dir if args.label_dir is not None else args.src_root / "kitti_labels"
    dst_root = args.dst_root
    points_dir = dst_root / "points"
    labels_dir = dst_root / "labels"
    imagesets_dir = dst_root / "ImageSets"

    ensure_dir(points_dir)
    ensure_dir(labels_dir)
    ensure_dir(imagesets_dir)

    class_whitelist = set(args.classes) if args.classes else None
    pairs = collect_source_pairs(pcd_dir, label_dir)
    if args.limit is not None:
        pairs = pairs[: args.limit]

    all_class_names = set()
    converted_ids = []

    for sample_id, pcd_path, label_path in pairs:
        dst_points = points_dir / f"{sample_id}.npy"
        dst_label = labels_dir / f"{sample_id}.txt"

        if not dst_points.exists() or args.overwrite:
            point_features = read_pcd_with_fields(pcd_path, fake_intensity=args.fake_intensity)
            np.save(dst_points, point_features.astype(np.float32, copy=False))

        label_classes = convert_label_file(label_path, dst_label, class_whitelist, overwrite=args.overwrite)
        if class_whitelist is not None and not label_classes:
            # Keep only samples that still contain at least one object after filtering.
            if dst_points.exists() and dst_label.exists():
                dst_points.unlink(missing_ok=True)
                dst_label.unlink(missing_ok=True)
            continue

        all_class_names.update(label_classes)
        converted_ids.append(sample_id)

    train_ids, val_ids = split_sample_ids(converted_ids, args.train_ratio, args.shuffle, args.seed)
    write_split_file(imagesets_dir / "train.txt", train_ids)
    write_split_file(imagesets_dir / "val.txt", val_ids)

    print("Conversion finished.")
    print(f"Source PCD dir:   {pcd_dir}")
    print(f"Source label dir: {label_dir}")
    print(f"Destination:      {dst_root}")
    print(f"Converted files:  {len(converted_ids)}")
    print(f"Train samples:    {len(train_ids)}")
    print(f"Val samples:      {len(val_ids)}")
    print(f"Classes:          {sorted(all_class_names)}")
    print("")
    print("Next steps:")
    print("1. Adjust tools/cfgs/dataset_configs/custom_dataset.yaml to match your classes.")
    print("2. Generate custom infos with pcdet/datasets/custom/custom_dataset.py.")
    print("3. Train with a custom-model config.")


if __name__ == "__main__":
    main()
