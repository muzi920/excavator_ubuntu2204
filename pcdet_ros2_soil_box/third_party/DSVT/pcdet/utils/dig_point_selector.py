"""Interpretable digging-point post-processing for OpenPCDet predictions.

The selector operates only on the original point cloud and one OpenPCDet box.
It does not modify detector inputs, network outputs, or point features beyond XYZ.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np


ArrayLike = Any
OPENPCDET_CPU_BOX_XY_MARGIN = 1e-2


@dataclass
class DigPointConfig:
    """All tunable thresholds used by :class:`DigPointSelector`."""

    score_threshold: float = 0.50
    grid_resolution: float = 0.05

    min_cloud_points: int = 20
    min_points_per_cell: int = 3
    min_pile_height: float = 0.20
    z_low_percentile: float = 5.0
    z_high_percentile: float = 95.0

    min_height_ratio: float = 0.40
    max_height_ratio: float = 0.85
    target_height_ratio: float = 0.65
    height_tolerance: float = 0.30

    min_front_ratio: float = 0.40
    target_front_ratio: float = 0.75
    front_tolerance: float = 0.35
    front_direction_epsilon: float = 1e-6
    front_low_percentile: float = 5.0
    front_high_percentile: float = 95.0

    fallback_min_height_ratio: float = 0.30
    fallback_max_height_ratio: float = 0.90
    fallback_min_front_ratio: float = 0.25

    density_radius: float = 0.25
    history_sigma: float = 0.40

    w_height: float = 0.35
    w_front: float = 0.35
    w_density: float = 0.30
    w_history: float = 0.35

    baseline_depth_offset: float = 0.15
    baseline_top_percent: float = 1.0
    baseline_search_radius: float = 0.25

    debug_top_k: int = 20
    verbose: bool = False

    def __post_init__(self) -> None:
        positive = {
            "grid_resolution": self.grid_resolution,
            "min_cloud_points": self.min_cloud_points,
            "min_points_per_cell": self.min_points_per_cell,
            "min_pile_height": self.min_pile_height,
            "height_tolerance": self.height_tolerance,
            "front_tolerance": self.front_tolerance,
            "front_direction_epsilon": self.front_direction_epsilon,
            "density_radius": self.density_radius,
            "history_sigma": self.history_sigma,
            "baseline_top_percent": self.baseline_top_percent,
            "baseline_search_radius": self.baseline_search_radius,
            "debug_top_k": self.debug_top_k,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")
        if not 0 <= self.score_threshold <= 1:
            raise ValueError("score_threshold must be in [0, 1]")
        if not 0 <= self.z_low_percentile < self.z_high_percentile <= 100:
            raise ValueError("z percentiles must satisfy 0 <= low < high <= 100")
        if not 0 <= self.front_low_percentile < self.front_high_percentile <= 100:
            raise ValueError("front percentiles must satisfy 0 <= low < high <= 100")
        if not 0 <= self.min_height_ratio <= self.max_height_ratio <= 1:
            raise ValueError("height-ratio limits must satisfy 0 <= min <= max <= 1")
        if not 0 <= self.fallback_min_height_ratio <= self.fallback_max_height_ratio <= 1:
            raise ValueError("fallback height-ratio limits must satisfy 0 <= min <= max <= 1")
        if not 0 <= self.min_front_ratio <= 1:
            raise ValueError("min_front_ratio must be in [0, 1]")
        if not 0 <= self.fallback_min_front_ratio <= 1:
            raise ValueError("fallback_min_front_ratio must be in [0, 1]")
        if not 0 < self.baseline_top_percent <= 100:
            raise ValueError("baseline_top_percent must be in (0, 100]")
        if self.baseline_depth_offset < 0:
            raise ValueError("baseline_depth_offset must be non-negative")
        for name in ("target_height_ratio", "target_front_ratio"):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in [0, 1]")
        for name in ("w_height", "w_front", "w_density", "w_history"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be non-negative")


def _as_numpy(value: ArrayLike, dtype: Optional[np.dtype] = None) -> np.ndarray:
    """Convert NumPy arrays or torch tensors without making torch a dependency."""
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=dtype)


def _numpy_points_in_box(points_xyz: np.ndarray, box: np.ndarray) -> np.ndarray:
    """Dependency-free fallback matching OpenPCDet's center-based box convention."""
    offset = points_xyz - box[:3]
    cos_h = np.cos(box[6])
    sin_h = np.sin(box[6])
    # Rotate world offsets by -heading into the local box frame.
    local_x = offset[:, 0] * cos_h + offset[:, 1] * sin_h
    local_y = -offset[:, 0] * sin_h + offset[:, 1] * cos_h
    return (
        (np.abs(local_x) < box[3] * 0.5 + OPENPCDET_CPU_BOX_XY_MARGIN)
        & (np.abs(local_y) < box[4] * 0.5 + OPENPCDET_CPU_BOX_XY_MARGIN)
        & (np.abs(offset[:, 2]) <= box[5] * 0.5)
    )


def points_in_box_mask(points_xyz: ArrayLike, box: ArrayLike) -> Tuple[np.ndarray, str]:
    """Return a rotated-box mask, preferring OpenPCDet's compiled implementation.

    The NumPy path is intentionally only a fallback for lightweight tests or an
    environment where the OpenPCDet extension has not yet been compiled.
    """
    xyz = _as_numpy(points_xyz, np.float32)
    box_array = _as_numpy(box, np.float32).reshape(-1)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    if box_array.size < 7:
        raise ValueError("box must contain at least 7 values")
    if not np.isfinite(box_array[:7]).all():
        raise ValueError("box contains NaN or infinity")
    if np.any(box_array[3:6] <= 0):
        raise ValueError("box dimensions dx, dy, dz must be positive")
    box7 = np.ascontiguousarray(box_array[:7].reshape(1, 7), dtype=np.float32)
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)

    try:
        from pcdet.ops.roiaware_pool3d import roiaware_pool3d_utils

        mask = roiaware_pool3d_utils.points_in_boxes_cpu(xyz, box7)[0] > 0
        return np.asarray(mask, dtype=bool), "openpcdet_roiaware_pool3d"
    except (ImportError, ModuleNotFoundError, OSError, RuntimeError, AttributeError):
        return _numpy_points_in_box(xyz, box7[0]), "numpy_fallback"


def _invalid_result(reason: str, **extra: Any) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "valid": False,
        "reason": reason,
        "dig_point": None,
        "score": None,
        "debug_candidates": [],
    }
    result.update(extra)
    return result


class DigPointSelector:
    """Select a real LiDAR point on the front, upper-middle part of a pile."""

    def __init__(self, config: Optional[DigPointConfig] = None) -> None:
        self.config = config if config is not None else DigPointConfig()

    def extract_pile_points(
        self, points: ArrayLike, box: ArrayLike
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        """Crop full-feature points by a rotated 3D box.

        Returns the cropped points, their indices in the input, and the mask
        backend name. Only XYZ participates in geometry; extra features remain.
        """
        point_array = _as_numpy(points)
        box_array = _as_numpy(box, np.float32).reshape(-1)
        if point_array.ndim != 2 or point_array.shape[1] < 3:
            raise ValueError("points must have shape (N, >=3)")
        if box_array.size < 7:
            raise ValueError("box must contain [x, y, z, dx, dy, dz, heading]")
        if not np.isfinite(box_array[:7]).all():
            raise ValueError("box contains NaN or infinity")
        if np.any(box_array[3:6] <= 0):
            raise ValueError("box dimensions dx, dy, dz must be positive")

        finite_xyz = np.isfinite(point_array[:, :3]).all(axis=1)
        finite_indices = np.flatnonzero(finite_xyz)
        mask, backend = points_in_box_mask(point_array[finite_xyz, :3], box_array[:7])
        input_indices = finite_indices[mask]
        return point_array[input_indices], input_indices, backend

    def _build_height_map(
        self, pile_points: np.ndarray
    ) -> Tuple[Dict[Tuple[int, int], Dict[str, Any]], np.ndarray]:
        resolution = self.config.grid_resolution
        origin_xy = np.min(pile_points[:, :2], axis=0)
        grid_indices = np.floor((pile_points[:, :2] - origin_xy) / resolution).astype(np.int64)

        grouped: Dict[Tuple[int, int], List[int]] = {}
        for point_index, (ix, iy) in enumerate(grid_indices):
            grouped.setdefault((int(ix), int(iy)), []).append(point_index)

        cells: Dict[Tuple[int, int], Dict[str, Any]] = {}
        for (ix, iy), indices_list in grouped.items():
            indices = np.asarray(indices_list, dtype=np.int64)
            z_values = pile_points[indices, 2]
            cells[(ix, iy)] = {
                "ix": ix,
                "iy": iy,
                "center_x": float(origin_xy[0] + (ix + 0.5) * resolution),
                "center_y": float(origin_xy[1] + (iy + 0.5) * resolution),
                "point_indices": indices,
                "point_count": int(indices.size),
                "mean_z": float(np.mean(z_values)),
                "median_z": float(np.median(z_values)),
                "max_z": float(np.max(z_values)),
            }
        return cells, origin_xy

    def _neighbor_count(
        self,
        key: Tuple[int, int],
        cells: Mapping[Tuple[int, int], Mapping[str, Any]],
    ) -> int:
        resolution = self.config.grid_resolution
        radius = self.config.density_radius
        cell_radius = int(np.ceil(radius / resolution))
        radius_squared = radius * radius + 1e-12
        ix, iy = key
        count = 0
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                neighbor_key = (ix + dx, iy + dy)
                if neighbor_key not in cells:
                    continue
                if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= radius_squared:
                    count += 1
        return count

    def _make_candidates(
        self,
        cells: Mapping[Tuple[int, int], Dict[str, Any]],
        z_low: float,
        pile_height: float,
        front_low: float,
        front_span: float,
        pile_center_xy: np.ndarray,
        front_direction: np.ndarray,
        last_dig_point: Optional[np.ndarray],
        min_height_ratio: float,
        max_height_ratio: float,
        min_front_ratio: float,
    ) -> List[Dict[str, Any]]:
        config = self.config
        candidates: List[Dict[str, Any]] = []
        for key, cell in cells.items():
            if cell["point_count"] < config.min_points_per_cell:
                continue
            cell_xy = np.array([cell["center_x"], cell["center_y"]], dtype=np.float64)
            front_projection = float(np.dot(cell_xy - pile_center_xy, front_direction))
            front_ratio = float(np.clip((front_projection - front_low) / front_span, 0.0, 1.0))
            height_ratio = float((cell["median_z"] - z_low) / pile_height)
            if not min_height_ratio <= height_ratio <= max_height_ratio:
                continue
            if front_ratio < min_front_ratio:
                continue

            height_score = float(np.clip(
                1.0 - abs(height_ratio - config.target_height_ratio) / config.height_tolerance,
                0.0,
                1.0,
            ))
            front_score = float(np.clip(
                1.0 - abs(front_ratio - config.target_front_ratio) / config.front_tolerance,
                0.0,
                1.0,
            ))
            if last_dig_point is None:
                history_penalty = 0.0
            else:
                history_distance = float(np.linalg.norm(cell_xy - last_dig_point[:2]))
                history_penalty = float(np.exp(
                    -(history_distance ** 2) / (2.0 * config.history_sigma ** 2)
                ))

            candidates.append({
                "_cell_key": key,
                "position": [cell["center_x"], cell["center_y"], cell["median_z"]],
                "height_ratio": height_ratio,
                "front_ratio": front_ratio,
                "height_score": height_score,
                "front_score": front_score,
                "density_score": 0.0,
                "history_penalty": history_penalty,
                "score": 0.0,
                "point_count": cell["point_count"],
                "neighbor_cell_count": self._neighbor_count(key, cells),
            })

        max_neighbor_count = max(
            (candidate["neighbor_cell_count"] for candidate in candidates), default=0
        )
        for candidate in candidates:
            if max_neighbor_count > 0:
                candidate["density_score"] = float(
                    candidate["neighbor_cell_count"] / max_neighbor_count
                )
            candidate["score"] = float(
                config.w_height * candidate["height_score"]
                + config.w_front * candidate["front_score"]
                + config.w_density * candidate["density_score"]
                - config.w_history * candidate["history_penalty"]
            )
        candidates.sort(key=lambda item: item["score"], reverse=True)
        return candidates

    @staticmethod
    def _public_candidate(candidate: Mapping[str, Any]) -> Dict[str, Any]:
        return {key: value for key, value in candidate.items() if not key.startswith("_")}

    def _fallback_real_point(
        self,
        pile_points: np.ndarray,
        cells: Mapping[Tuple[int, int], Dict[str, Any]],
        origin_xy: np.ndarray,
        pile_center_xy: np.ndarray,
        front_direction: np.ndarray,
        front_low: float,
        front_span: float,
        z_low: float,
        pile_height: float,
        last_dig_point: Optional[np.ndarray],
    ) -> Optional[Tuple[np.ndarray, Dict[str, Any]]]:
        projections = (pile_points[:, :2] - pile_center_xy) @ front_direction
        front_mask = projections > np.median(projections)
        front_indices = np.flatnonzero(front_mask)
        if front_indices.size == 0:
            return None

        target_z = z_low + self.config.target_height_ratio * pile_height
        point_index = front_indices[np.argmin(np.abs(pile_points[front_indices, 2] - target_z))]
        dig_point = pile_points[point_index, :3].astype(np.float64, copy=True)
        point_projection = float(projections[point_index])
        front_ratio = float(np.clip((point_projection - front_low) / front_span, 0.0, 1.0))
        height_ratio = float((dig_point[2] - z_low) / pile_height)
        height_score = float(np.clip(
            1.0 - abs(height_ratio - self.config.target_height_ratio) / self.config.height_tolerance,
            0.0,
            1.0,
        ))
        front_score = float(np.clip(
            1.0 - abs(front_ratio - self.config.target_front_ratio) / self.config.front_tolerance,
            0.0,
            1.0,
        ))
        if last_dig_point is None:
            history_penalty = 0.0
        else:
            distance = float(np.linalg.norm(dig_point[:2] - last_dig_point[:2]))
            history_penalty = float(np.exp(
                -(distance ** 2) / (2.0 * self.config.history_sigma ** 2)
            ))

        cell_index = np.floor(
            (dig_point[:2] - origin_xy) / self.config.grid_resolution
        ).astype(np.int64)
        cell_key = (int(cell_index[0]), int(cell_index[1]))
        neighbor_count = self._neighbor_count(cell_key, cells) if cell_key in cells else 0
        max_possible_neighbors = max(
            (self._neighbor_count(key, cells) for key in cells), default=0
        )
        density_score = float(neighbor_count / max_possible_neighbors) if max_possible_neighbors else 0.0
        score = float(
            self.config.w_height * height_score
            + self.config.w_front * front_score
            + self.config.w_density * density_score
            - self.config.w_history * history_penalty
        )
        candidate = {
            "position": dig_point.tolist(),
            "height_ratio": height_ratio,
            "front_ratio": front_ratio,
            "height_score": height_score,
            "front_score": front_score,
            "density_score": density_score,
            "history_penalty": history_penalty,
            "score": score,
            "point_count": int(cells[cell_key]["point_count"]) if cell_key in cells else 1,
            "neighbor_cell_count": neighbor_count,
        }
        return dig_point, candidate

    def select(
        self,
        points: ArrayLike,
        box: ArrayLike,
        excavator_position: Optional[ArrayLike] = None,
        last_dig_point: Optional[ArrayLike] = None,
    ) -> Dict[str, Any]:
        """Select a digging point for one predicted pile box.

        ``excavator_position`` is explicit at every call. Passing ``None`` is
        treated as invalid so that a coordinate-frame assumption cannot become
        hidden inside the algorithm.
        """
        try:
            point_array = _as_numpy(points)
            box_array = _as_numpy(box, np.float64).reshape(-1)
            if point_array.ndim != 2 or point_array.shape[1] < 3:
                return _invalid_result("points_must_have_shape_N_by_at_least_3")
            if box_array.size < 7:
                return _invalid_result("box_must_have_at_least_7_values")
            if excavator_position is None:
                return _invalid_result("excavator_position_is_required")
            excavator = _as_numpy(excavator_position, np.float64).reshape(-1)
            if excavator.size < 3 or not np.isfinite(excavator[:3]).all():
                return _invalid_result("invalid_excavator_position")
            previous = None
            if last_dig_point is not None:
                previous = _as_numpy(last_dig_point, np.float64).reshape(-1)
                if previous.size < 2 or not np.isfinite(previous[:2]).all():
                    return _invalid_result("invalid_last_dig_point")

            pile_points, input_indices, mask_backend = self.extract_pile_points(
                point_array, box_array
            )
        except (TypeError, ValueError, IndexError) as exc:
            return _invalid_result(f"invalid_input: {exc}")

        if pile_points.shape[0] < self.config.min_cloud_points:
            return _invalid_result(
                "too_few_pile_points",
                num_pile_points=int(pile_points.shape[0]),
                min_cloud_points=self.config.min_cloud_points,
            )

        z_values = pile_points[:, 2].astype(np.float64, copy=False)
        z_low, z_high = np.percentile(
            z_values, [self.config.z_low_percentile, self.config.z_high_percentile]
        )
        pile_height = float(z_high - z_low)
        if not np.isfinite(pile_height) or pile_height < self.config.min_pile_height:
            return _invalid_result(
                "pile_height_below_threshold",
                num_pile_points=int(pile_points.shape[0]),
                pile_height=pile_height,
                min_pile_height=self.config.min_pile_height,
            )

        pile_center_xy = np.median(pile_points[:, :2], axis=0).astype(np.float64)
        front_vector = excavator[:2] - pile_center_xy
        front_distance = float(np.linalg.norm(front_vector))
        if front_distance < self.config.front_direction_epsilon:
            return _invalid_result(
                "excavator_too_close_to_pile_center_for_front_direction",
                num_pile_points=int(pile_points.shape[0]),
                pile_center_xy=pile_center_xy,
            )
        front_direction = front_vector / front_distance

        cells, origin_xy = self._build_height_map(pile_points)
        cell_centers = np.asarray(
            [[cell["center_x"], cell["center_y"]] for cell in cells.values()],
            dtype=np.float64,
        )
        cell_projections = (cell_centers - pile_center_xy) @ front_direction
        front_low, front_high = np.percentile(
            cell_projections,
            [self.config.front_low_percentile, self.config.front_high_percentile],
        )
        front_span = float(front_high - front_low)
        if front_span < self.config.front_direction_epsilon:
            return _invalid_result(
                "pile_has_insufficient_front_extent",
                num_pile_points=int(pile_points.shape[0]),
                num_grid_cells=len(cells),
            )

        candidates = self._make_candidates(
            cells, float(z_low), pile_height, float(front_low), front_span,
            pile_center_xy, front_direction, previous,
            self.config.min_height_ratio, self.config.max_height_ratio,
            self.config.min_front_ratio,
        )
        fallback_level = 0
        if not candidates:
            fallback_level = 1
            candidates = self._make_candidates(
                cells, float(z_low), pile_height, float(front_low), front_span,
                pile_center_xy, front_direction, previous,
                self.config.fallback_min_height_ratio,
                self.config.fallback_max_height_ratio,
                self.config.fallback_min_front_ratio,
            )

        if candidates:
            best = candidates[0]
            best_cell = cells[best["_cell_key"]]
            cell_points = pile_points[best_cell["point_indices"], :3]
            target = np.asarray(best["position"], dtype=np.float64)
            local_index = int(np.argmin(np.linalg.norm(cell_points - target, axis=1)))
            dig_point = cell_points[local_index].astype(np.float64, copy=True)
        else:
            fallback_level = 2
            fallback = self._fallback_real_point(
                pile_points, cells, origin_xy, pile_center_xy, front_direction,
                float(front_low), front_span, float(z_low), pile_height, previous,
            )
            if fallback is None:
                return _invalid_result(
                    "no_candidate_after_all_fallbacks",
                    num_pile_points=int(pile_points.shape[0]),
                    num_grid_cells=len(cells),
                )
            dig_point, best = fallback
            candidates = [best]

        public_candidates = [
            self._public_candidate(candidate)
            for candidate in candidates[: self.config.debug_top_k]
        ]
        result: Dict[str, Any] = {
            "valid": True,
            "dig_point": dig_point,
            "score": float(best["score"]),
            "height_score": float(best["height_score"]),
            "front_score": float(best["front_score"]),
            "density_score": float(best["density_score"]),
            "history_penalty": float(best["history_penalty"]),
            "num_pile_points": int(pile_points.shape[0]),
            "num_candidates": len(candidates),
            "debug_candidates": public_candidates,
            "debug": {
                "method": "front_height_density",
                "fallback_level": fallback_level,
                "pile_points": pile_points,
                "pile_point_input_indices": input_indices,
                "points_in_box_backend": mask_backend,
                "pile_height": pile_height,
                "z_low": float(z_low),
                "z_high": float(z_high),
                "num_grid_cells": len(cells),
                "pile_center_xy": pile_center_xy,
                "excavator_position": excavator[:3].copy(),
                "front_direction": front_direction,
                "front_low": float(front_low),
                "front_high": float(front_high),
            },
        }
        if self.config.verbose:
            self._print_result(result)
        return result

    @staticmethod
    def _print_result(result: Mapping[str, Any]) -> None:
        if not result.get("valid", False):
            print("========== Dig Point Selection ==========")
            print(f"Invalid: {result.get('reason', 'unknown reason')}")
            print("=========================================")
            return
        debug = result["debug"]
        dig_point = result["dig_point"]
        print("========== Dig Point Selection ==========")
        print(f"\nPile points: {result['num_pile_points']}")
        print(f"Pile height: {debug['pile_height']:.3f} m")
        print(f"\nGrid cells: {debug['num_grid_cells']}")
        print(f"Candidate cells: {result['num_candidates']}")
        print(f"Fallback level: {debug['fallback_level']}")
        print(f"\nPile center:\n{np.array2string(debug['pile_center_xy'], precision=3)}")
        print(f"\nExcavator position:\n{np.array2string(debug['excavator_position'], precision=3)}")
        print(f"\nFront direction:\n{np.array2string(debug['front_direction'], precision=3)}")
        print("\nBest candidate:")
        print(f"\nXYZ:\n{np.array2string(dig_point, precision=3)}")
        best = result["debug_candidates"][0]
        print(f"\nheight_ratio: {best['height_ratio']:.3f}")
        print(f"front_ratio: {best['front_ratio']:.3f}")
        print(f"\nheight_score: {result['height_score']:.3f}")
        print(f"front_score: {result['front_score']:.3f}")
        print(f"density_score: {result['density_score']:.3f}")
        print(f"history_penalty: {result['history_penalty']:.3f}")
        print(f"\nfinal_score: {result['score']:.3f}\n")
        print("=========================================")

    def save_debug(self, result: Mapping[str, Any], output_dir: ArrayLike) -> None:
        """Save the selected point, ranked candidates, and cropped full-feature cloud."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        if not result.get("valid", False):
            (output_path / "invalid_reason.txt").write_text(
                str(result.get("reason", "unknown reason")) + "\n", encoding="utf-8"
            )
            return

        dig_point = np.asarray(result["dig_point"], dtype=np.float64)
        np.savetxt(
            output_path / "dig_point.txt",
            np.asarray([[dig_point[0], dig_point[1], dig_point[2], result["score"]]]),
            fmt="%.8f",
            header="x y z score",
            comments="",
        )
        fieldnames = [
            "x", "y", "z", "height_ratio", "front_ratio", "height_score",
            "front_score", "density_score", "history_penalty", "score",
            "point_count", "neighbor_cell_count",
        ]
        with (output_path / "candidates.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for candidate in result.get("debug_candidates", []):
                position = candidate["position"]
                row = {"x": position[0], "y": position[1], "z": position[2]}
                row.update({name: candidate[name] for name in fieldnames[3:]})
                writer.writerow(row)
        np.save(output_path / "pile_points.npy", result["debug"]["pile_points"])


def select_baseline_dig_point(
    points: ArrayLike,
    box: Optional[ArrayLike] = None,
    baseline_depth_offset: float = 0.15,
    search_radius: float = 0.25,
    top_percent: float = 1.0,
    min_cloud_points: int = 20,
    z_low_percentile: float = 5.0,
    z_high_percentile: float = 95.0,
) -> Dict[str, Any]:
    """Select a real point near the top-region center and below robust ``z_high``.

    If ``box`` is ``None``, ``points`` is assumed to already contain pile points.
    Otherwise a rotated crop is performed with the same OpenPCDet-first logic as
    the scored selector.
    """
    if (
        baseline_depth_offset < 0
        or search_radius <= 0
        or not 0 < top_percent <= 100
        or not 0 <= z_low_percentile < z_high_percentile <= 100
    ):
        return _invalid_result("invalid_baseline_configuration")
    point_array = _as_numpy(points)
    if point_array.ndim != 2 or point_array.shape[1] < 3:
        return _invalid_result("points_must_have_shape_N_by_at_least_3")
    finite_points = point_array[np.isfinite(point_array[:, :3]).all(axis=1)]
    backend = "already_cropped"
    if box is not None:
        try:
            box_array = _as_numpy(box, np.float64).reshape(-1)
            mask, backend = points_in_box_mask(finite_points[:, :3], box_array)
            finite_points = finite_points[mask]
        except (TypeError, ValueError, IndexError) as exc:
            return _invalid_result(f"invalid_input: {exc}")
    if finite_points.shape[0] < min_cloud_points:
        return _invalid_result(
            "too_few_pile_points", num_pile_points=int(finite_points.shape[0])
        )

    z_values = finite_points[:, 2].astype(np.float64, copy=False)
    z_low, z_high = np.percentile(z_values, [z_low_percentile, z_high_percentile])
    top_threshold = float(np.percentile(z_values, 100.0 - top_percent))
    top_points = finite_points[z_values >= top_threshold]
    if top_points.shape[0] == 0:
        return _invalid_result("no_points_in_top_region")
    top_center_xy = np.median(top_points[:, :2], axis=0)
    target_z = float(z_high - baseline_depth_offset)
    xy_distances = np.linalg.norm(finite_points[:, :2] - top_center_xy, axis=1)
    nearby_indices = np.flatnonzero(xy_distances <= search_radius)
    if nearby_indices.size == 0:
        # A sparse scan may contain no return inside the configured disk. Keep
        # the baseline usable by taking the closest XY point as its final pool.
        nearby_indices = np.asarray([int(np.argmin(xy_distances))], dtype=np.int64)
    height_errors = np.abs(finite_points[nearby_indices, 2] - target_z)
    # Height error is primary; XY distance only resolves equal-height choices.
    ranked_local_indices = np.lexsort((xy_distances[nearby_indices], height_errors))
    best_index = int(nearby_indices[ranked_local_indices[0]])
    dig_point = finite_points[best_index, :3].astype(np.float64, copy=True)
    pile_height = max(float(z_high - z_low), np.finfo(np.float64).eps)
    score = float(np.clip(1.0 - abs(dig_point[2] - target_z) / pile_height, 0.0, 1.0))
    candidate = {
        "position": dig_point.tolist(),
        "height_ratio": float((dig_point[2] - z_low) / pile_height),
        "front_ratio": 0.0,
        "height_score": score,
        "front_score": 0.0,
        "density_score": 0.0,
        "history_penalty": 0.0,
        "score": score,
        "point_count": int(nearby_indices.size),
        "neighbor_cell_count": 0,
    }
    return {
        "valid": True,
        "dig_point": dig_point,
        "score": score,
        "height_score": score,
        "front_score": 0.0,
        "density_score": 0.0,
        "history_penalty": 0.0,
        "num_pile_points": int(finite_points.shape[0]),
        "num_candidates": int(nearby_indices.size),
        "debug_candidates": [candidate],
        "debug": {
            "method": "top_region_baseline",
            "pile_points": finite_points,
            "points_in_box_backend": backend,
            "z_low": float(z_low),
            "z_high": float(z_high),
            "pile_height": float(z_high - z_low),
            "top_center_xy": top_center_xy,
            "top_threshold": top_threshold,
            "target_z": target_z,
            "baseline_depth_offset": baseline_depth_offset,
        },
    }


def resolve_target_label(
    class_names: Sequence[str], target_class_name: Optional[str] = None
) -> Tuple[int, str]:
    """Resolve a 1-based OpenPCDet label from the active dataset class list."""
    names = [str(name) for name in class_names]
    if not names:
        raise ValueError("class_names is empty")
    if target_class_name is None:
        if len(names) != 1:
            raise ValueError(
                "target_class_name is required when the dataset has multiple classes"
            )
        return 1, names[0]
    matches = [index for index, name in enumerate(names) if name.casefold() == target_class_name.casefold()]
    if len(matches) != 1:
        raise ValueError(
            f"target class {target_class_name!r} not found uniquely in {names}"
        )
    return matches[0] + 1, names[matches[0]]


def select_dig_points_from_predictions(
    selector: DigPointSelector,
    points: ArrayLike,
    pred_dict: Mapping[str, ArrayLike],
    class_names: Sequence[str],
    excavator_position: ArrayLike,
    last_dig_point: Optional[ArrayLike] = None,
    target_class_name: Optional[str] = None,
    score_threshold: Optional[float] = None,
    method: str = "scored",
) -> List[Dict[str, Any]]:
    """Run digging-point selection independently for every matching detection."""
    required_keys = ("pred_boxes", "pred_scores", "pred_labels")
    missing = [key for key in required_keys if key not in pred_dict]
    if missing:
        raise KeyError(f"pred_dict is missing keys: {missing}")
    boxes = _as_numpy(pred_dict["pred_boxes"], np.float64)
    scores = _as_numpy(pred_dict["pred_scores"], np.float64).reshape(-1)
    labels = _as_numpy(pred_dict["pred_labels"]).reshape(-1).astype(np.int64)
    if boxes.ndim != 2 or boxes.shape[1] < 7:
        raise ValueError("pred_boxes must have shape (N, >=7)")
    if not (boxes.shape[0] == scores.size == labels.size):
        raise ValueError("pred_boxes, pred_scores, and pred_labels lengths differ")

    target_label, resolved_name = resolve_target_label(class_names, target_class_name)
    threshold = selector.config.score_threshold if score_threshold is None else float(score_threshold)
    if not 0 <= threshold <= 1:
        raise ValueError("score_threshold must be in [0, 1]")
    if method not in {"scored", "baseline"}:
        raise ValueError("method must be 'scored' or 'baseline'")

    detection_indices = np.flatnonzero((labels == target_label) & (scores >= threshold))
    detection_indices = detection_indices[np.argsort(-scores[detection_indices])]
    results: List[Dict[str, Any]] = []
    for index in detection_indices:
        box = boxes[index].copy()
        if method == "baseline":
            selection = select_baseline_dig_point(
                points,
                box,
                baseline_depth_offset=selector.config.baseline_depth_offset,
                search_radius=selector.config.baseline_search_radius,
                top_percent=selector.config.baseline_top_percent,
                min_cloud_points=selector.config.min_cloud_points,
                z_low_percentile=selector.config.z_low_percentile,
                z_high_percentile=selector.config.z_high_percentile,
            )
        else:
            selection = selector.select(
                points, box, excavator_position=excavator_position,
                last_dig_point=last_dig_point,
            )
        results.append({
            "box": box,
            "score": float(scores[index]),
            "label": int(labels[index]),
            "class_name": resolved_name,
            "valid": bool(selection["valid"]),
            "dig_point": selection.get("dig_point"),
            "dig_score": selection.get("score"),
            "selection": selection,
        })
    return results


def select_highest_confidence_pile(
    results: Sequence[Mapping[str, Any]], valid_only: bool = True
) -> Optional[Mapping[str, Any]]:
    """Return the pile result with the highest detector confidence."""
    eligible = [result for result in results if result.get("valid", False) or not valid_only]
    return max(eligible, key=lambda result: float(result["score"])) if eligible else None


def visualize_dig_point(
    pile_points: ArrayLike,
    result: Mapping[str, Any],
    show: bool = True,
    ax: Optional[Any] = None,
) -> Tuple[Any, Any]:
    """Optional Matplotlib XY debug view; the selection algorithm does not import it."""
    import matplotlib.pyplot as plt

    points = _as_numpy(pile_points)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("pile_points must have shape (N, >=3)")
    if ax is None:
        figure, ax = plt.subplots(figsize=(8, 7))
    else:
        figure = ax.figure
    cloud = ax.scatter(points[:, 0], points[:, 1], c=points[:, 2], s=2, cmap="terrain", alpha=0.6)
    figure.colorbar(cloud, ax=ax, label="z (m)")

    candidates = result.get("debug_candidates", [])
    if candidates:
        candidate_xy = np.asarray([candidate["position"][:2] for candidate in candidates])
        candidate_scores = np.asarray([candidate["score"] for candidate in candidates])
        ax.scatter(
            candidate_xy[:, 0], candidate_xy[:, 1], c=candidate_scores,
            s=35, cmap="viridis", edgecolors="black", linewidths=0.3,
            label="candidate cells",
        )
    if result.get("valid", False):
        dig_point = np.asarray(result["dig_point"])
        ax.scatter(
            dig_point[0], dig_point[1], marker="*", s=320, c="red",
            edgecolors="black", linewidths=1.5, zorder=10, label="dig point",
        )
    excavator = result.get("debug", {}).get("excavator_position")
    if excavator is not None:
        excavator = np.asarray(excavator)
        ax.scatter(excavator[0], excavator[1], marker="s", s=100, c="blue", label="excavator")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Digging-point selection (XY)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    figure.tight_layout()
    if show:
        plt.show()
    return figure, ax


__all__ = [
    "DigPointConfig",
    "DigPointSelector",
    "points_in_box_mask",
    "resolve_target_label",
    "select_baseline_dig_point",
    "select_dig_points_from_predictions",
    "select_highest_confidence_pile",
    "visualize_dig_point",
]
