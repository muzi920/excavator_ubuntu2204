"""
无监督 LiDAR 点云目标检测器 — 纯 numpy RANSAC 地面分割 + 欧氏聚类。

不依赖 open3d/sklearn, 只需要 numpy + scipy.

用法:
    detector = ClusterDetector()
    boxes = detector.detect(points)
"""

import numpy as np
from scipy.spatial import KDTree


class ClusterDetector:
    def __init__(self, cluster_eps=0.5, min_points=10, ground_dist=0.2):
        """
        cluster_eps : float  聚类邻域半径 (m)
        min_points  : int    最小聚类点数
        ground_dist : float  RANSAC 地面距离阈值 (m)
        """
        self.cluster_eps = cluster_eps
        self.min_points = min_points
        self.ground_dist = ground_dist

    def detect(self, points):
        xyz = points[:, :3].copy()
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        if len(xyz) < self.min_points:
            return np.empty((0, 7)), np.empty((0,), dtype=np.int32)

        # ---- 1. 体素下采样 ----
        vc = np.floor(xyz / 0.3).astype(np.int64)
        _, u = np.unique(vc, axis=0, return_index=True)
        xyz_ds = xyz[u]

        # ---- 2. RANSAC 平面拟合 (纯 numpy) ----
        plane, ground_mask = self._ransac_plane(xyz_ds)
        non_ground = xyz_ds[~ground_mask]

        if len(non_ground) < self.min_points:
            return np.empty((0, 7)), np.empty((0,), dtype=np.int32)

        # ---- 3. KD-tree BFS 聚类 ----
        clusters = self._cluster(non_ground)

        # ---- 4. 包围盒 ----
        boxes = []
        for idx_list in clusters:
            if len(idx_list) < self.min_points:
                continue
            cpts = non_ground[idx_list]
            lo, hi = cpts.min(axis=0), cpts.max(axis=0)
            center = (lo + hi) / 2
            size = np.maximum(hi - lo, 0.15)
            boxes.append([*center, *size, 0.0])

        if not boxes:
            return np.empty((0, 7)), np.empty((0,), dtype=np.int32)

        return np.array(boxes, dtype=np.float32), np.zeros(len(boxes), dtype=np.int32)

    def _ransac_plane(self, points, n_iter=200):
        """RANSAC 拟合 ax+by+cz+d=0 平面, 返回 (plane, inlier_mask)."""
        best_inliers = 0
        best_mask = None
        best_plane = None
        N = len(points)

        for _ in range(n_iter):
            # 随机选 3 个点
            idx = np.random.choice(N, 3, replace=False)
            p1, p2, p3 = points[idx]
            # 法向量 = (p2-p1) × (p3-p1)
            n = np.cross(p2 - p1, p3 - p1)
            if np.linalg.norm(n) < 1e-6:
                continue
            n = n / np.linalg.norm(n)
            # a*x + b*y + c*z + d = 0  →  d = -n·p1
            d = -np.dot(n, p1)
            plane = np.append(n, d)

            # 计算所有点到平面的距离
            dists = np.abs(np.dot(points, n) + d)
            mask = dists < self.ground_dist
            n_inliers = mask.sum()

            if n_inliers > best_inliers:
                best_inliers = n_inliers
                best_mask = mask
                best_plane = plane

        return best_plane, best_mask

    def _cluster(self, points):
        """KD-tree BFS 连通分量聚类。"""
        tree = KDTree(points)
        n = len(points)
        visited = np.zeros(n, dtype=bool)
        all_clusters = []

        for i in range(n):
            if visited[i]:
                continue
            cluster = []
            queue = [i]
            visited[i] = True
            while queue:
                idx = queue.pop()
                cluster.append(idx)
                nbs = tree.query_ball_point(points[idx], self.cluster_eps)
                for nb in nbs:
                    if not visited[nb]:
                        visited[nb] = True
                        queue.append(nb)
            all_clusters.append(cluster)
        return all_clusters
