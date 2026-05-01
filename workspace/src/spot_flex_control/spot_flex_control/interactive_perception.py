"""Joint estimation and path-following state utilities ported from flex_spot.

Matches the iterative-probing variant of flex_spot/robot/flex/interactive_perception.py
(paired with the executor's estimate_trajectory_2d_via_probing). The module exposes:
    - generate_wiggle_positions: cardinal-direction wiggle (substep variant).
    - prismatic_error_analysis: line fit + perpendicular MSE.
    - fit_model_2d: 2D AIC-based line/circle classifier returning (model, confidence).
    - fit_circle_in_plane: 3D circle fit in a plane orthogonal to a known axis.
    - revolute_error_analysis: 3D PCA + circle-in-plane + radius clamp.
    - analyze_trajectory_and_estimate_joint: 2D-first then 3D fallback with heuristics.
    - construct_path_following_state: 8D state for the path-following revolute actor.
    - update_closest_path_index: windowed nearest-point lookup.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


def project_points_onto_plane(points, normal_vector, origin):
    normal = np.asarray(normal_vector, dtype=float)
    normal = normal / np.linalg.norm(normal)
    origin = np.asarray(origin, dtype=float)
    pts = np.asarray(points, dtype=float)
    distances = (pts - origin) @ normal
    return pts - np.outer(distances, normal)


class InteractivePerception:
    def __init__(self, movement_distance: float = 0.15):
        self.movement_distance = float(movement_distance)
        self.max_revolute_radius = 2.0
        self.small_motion_threshold = 0.10
        self.revolute_better_factor = 0.5
        self.joint_params: Optional[dict] = None
        self.joint_type: Optional[str] = None

    def generate_wiggle_positions(self, start_position: np.ndarray) -> list:
        start = np.asarray(start_position, dtype=float)
        positions = [start.copy()]
        d = self.movement_distance
        directions = [
            np.array([d, 0.0, 0.0]),
            np.array([-d, 0.0, 0.0]),
            np.array([0.0, d, 0.0]),
            np.array([0.0, -d, 0.0]),
        ]
        num_substeps = 5
        for direction in directions:
            for step in range(1, num_substeps + 1):
                increment = direction * (step / num_substeps)
                positions.append((start + increment).copy())
            positions.append(start.copy())
        return positions

    def prismatic_error_analysis(self, trajectory: np.ndarray):
        traj = np.asarray(trajectory, dtype=float)
        centroid = np.mean(traj, axis=0)
        centered = traj - centroid
        _, _, vt = np.linalg.svd(centered)
        axis = vt[0]
        axis = axis / np.linalg.norm(axis)
        proj_scalars = centered @ axis
        projections = np.outer(proj_scalars, axis)
        residuals = np.linalg.norm(centered - projections, axis=1)
        return float(np.mean(residuals ** 2)), axis

    def fit_model_2d(self, points_2d):
        from scipy.optimize import least_squares

        pts = np.asarray(points_2d, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            raise ValueError('points_2d must be (N, 2).')
        x_arr = pts[:, 0]
        y_arr = pts[:, 1]
        n = len(pts)

        span = float(np.hypot(x_arr[-1] - x_arr[0], y_arr[-1] - y_arr[0]))
        if n < 5 or span < 0.01:
            return ('UNDEFINED', None), 0.0

        x_mean = float(np.mean(x_arr))
        y_mean = float(np.mean(y_arr))
        _, _, vh = np.linalg.svd(np.vstack([x_arr - x_mean, y_arr - y_mean]).T)
        dir_x, dir_y = vh[0]
        normal_x, normal_y = -dir_y, dir_x
        line_residuals = (x_arr - x_mean) * normal_x + (y_arr - y_mean) * normal_y
        rss_line = float(np.sum(line_residuals ** 2))

        def circle_residuals(params, x_in, y_in):
            xc, yc, r = params
            return np.sqrt((x_in - xc) ** 2 + (y_in - yc) ** 2) - r

        initial_r = span / 2.0 + 0.1
        try:
            res = least_squares(circle_residuals, [x_mean, y_mean, initial_r], args=(x_arr, y_arr))
            rss_circle = float(np.sum(res.fun ** 2))
            circle_params = res.x
        except Exception:
            res = None
            rss_circle = float('inf')
            circle_params = None

        aic_line = n * math.log(rss_line / n + 1e-9) + 2 * 2
        aic_circle = n * math.log(rss_circle / n + 1e-9) + 2 * 3
        is_circular = aic_circle < (aic_line - 2.0)

        if not is_circular:
            rmse = math.sqrt(rss_line / n)
            score_fit = float(np.clip(1.0 - (rmse / 0.02), 0.0, 1.0))
            score_span = float(np.clip(span / 0.10, 0.0, 1.0))
            confidence = score_fit * score_span
            model = ('LINEAR', {
                'direction': np.array([dir_x, dir_y]),
                'point': np.array([x_mean, y_mean]),
            })
            return model, confidence

        if circle_params is None:
            return ('UNDEFINED', None), 0.0

        xc, yc, r_est = circle_params
        mse = rss_circle / max(n - 3, 1)
        try:
            j = res.jac
            cov = np.linalg.pinv(j.T @ j) * mse
            sigma_r = math.sqrt(cov[2, 2])
            score_math = float(np.clip(1.0 - (sigma_r / r_est), 0.0, 1.0))
        except Exception:
            score_math = 0.0

        angles = np.arctan2(y_arr - yc, x_arr - xc)
        angles = np.unwrap(angles)
        angle_span = float(np.abs(angles[-1] - angles[0]))
        score_span = float(np.clip(angle_span / 0.26, 0.0, 1.0))
        confidence = score_math * score_span
        model = ('CIRCULAR', {
            'center': np.array([xc, yc]),
            'radius': float(r_est),
        })
        return model, confidence

    def fit_circle_in_plane(self, points_3d, axis):
        from sklearn.decomposition import PCA
        from scipy.optimize import least_squares

        origin = np.zeros(3)
        projected = project_points_onto_plane(points_3d, axis, origin)
        pca2 = PCA(n_components=2).fit(projected)
        points_2d = pca2.transform(projected)

        def circle_residuals(c, pts):
            d = np.linalg.norm(pts - c, axis=1)
            return d - d.mean()

        c0 = points_2d.mean(axis=0)
        res = least_squares(circle_residuals, c0, args=(points_2d,))
        center_2d = res.x
        radii = np.linalg.norm(points_2d - center_2d, axis=1)
        radius = float(radii.mean())
        center_plane = pca2.inverse_transform(center_2d)
        return center_plane, radius

    def revolute_error_analysis(self, trajectory: np.ndarray):
        from sklearn.decomposition import PCA

        traj = np.asarray(trajectory, dtype=float)
        mean_world = np.mean(traj, axis=0)
        centered = traj - mean_world
        pca3 = PCA(n_components=3).fit(centered)
        axis = pca3.components_[-1]
        axis = axis / np.linalg.norm(axis)

        center_centered, radius = self.fit_circle_in_plane(centered, axis)
        radius = float(min(radius, self.max_revolute_radius))
        center_world = center_centered + mean_world

        distances = np.linalg.norm(traj - center_world, axis=1)
        mse = float(np.mean((distances - radius) ** 2))
        return mse, center_world, radius, axis

    def analyze_trajectory_and_estimate_joint(self, trajectory: np.ndarray):
        traj = np.asarray(trajectory, dtype=float)
        if traj.shape[0] < 5:
            raise ValueError('Trajectory too short; need at least 5 points.')

        pris_error, pris_axis = self.prismatic_error_analysis(traj)
        rev_error, rev_center, rev_radius, rev_axis = self.revolute_error_analysis(traj)

        model_2d, conf_2d = self.fit_model_2d(traj[:, :2])
        z_range = float(np.ptp(traj[:, 2])) if traj.shape[1] >= 3 else 0.0
        if z_range <= 0.5 and conf_2d >= 0.5 and model_2d[0] != 'UNDEFINED':
            if model_2d[0] == 'CIRCULAR':
                center_xy = model_2d[1]['center']
                radius_xy = float(model_2d[1]['radius'])
                center_world = np.array(
                    [center_xy[0], center_xy[1], float(np.mean(traj[:, 2]))]
                )
                d = np.linalg.norm(traj[:, :2] - center_xy, axis=1)
                mse = float(np.mean((d - radius_xy) ** 2))
                self.joint_type = 'revolute'
                self.joint_params = {
                    'axis': np.array([0.0, 0.0, 1.0]),
                    'center': center_world,
                    'radius': radius_xy,
                    'error': mse,
                }
                return self.joint_type, self.joint_params

            direction_xy = model_2d[1]['direction']
            axis = np.array([direction_xy[0], direction_xy[1], 0.0])
            axis = axis / np.linalg.norm(axis)
            self.joint_type = 'prismatic'
            self.joint_params = {'axis': axis, 'error': pris_error}
            return self.joint_type, self.joint_params

        if rev_radius >= self.max_revolute_radius - 1e-6:
            self.joint_type = 'prismatic'
            self.joint_params = {'axis': pris_axis, 'error': pris_error}
            return self.joint_type, self.joint_params

        traj_range = np.ptp(traj, axis=0)
        max_range = float(np.max(traj_range))
        if max_range < self.small_motion_threshold:
            self.joint_type = 'prismatic'
            self.joint_params = {'axis': pris_axis, 'error': pris_error}
            return self.joint_type, self.joint_params

        if rev_error < self.revolute_better_factor * pris_error:
            self.joint_type = 'revolute'
            self.joint_params = {
                'axis': rev_axis,
                'center': rev_center,
                'radius': rev_radius,
                'error': rev_error,
            }
        else:
            self.joint_type = 'prismatic'
            self.joint_params = {'axis': pris_axis, 'error': pris_error}
        return self.joint_type, self.joint_params

    def force_revolute(self, trajectory: np.ndarray):
        mse, center, radius, axis = self.revolute_error_analysis(trajectory)
        self.joint_type = 'revolute'
        self.joint_params = {
            'center': center,
            'radius': radius,
            'axis': axis,
            'error': mse,
        }
        return self.joint_type, self.joint_params

    def update_closest_path_index(self, current_pos, path_points, last_idx: int) -> int:
        path = np.asarray(path_points, dtype=float)
        cur = np.asarray(current_pos, dtype=float)
        search_start = max(0, int(last_idx) - 5)
        search_end = min(len(path), int(last_idx) + 20)
        if search_start >= search_end:
            return int(last_idx)
        window = path[search_start:search_end]
        dists = np.linalg.norm(window - cur, axis=1)
        return int(search_start + int(np.argmin(dists)))

    def construct_path_following_state(
        self,
        current_pos,
        path_points,
        current_yaw: float,
        closest_idx: int,
        velocity_2d=None,
    ) -> np.ndarray:
        path = np.asarray(path_points, dtype=float)
        cur = np.asarray(current_pos, dtype=float)
        idx = int(closest_idx)
        closest_point = path[idx]

        if idx < len(path) - 1:
            tangent = path[idx + 1] - closest_point
        elif idx > 0:
            tangent = closest_point - path[idx - 1]
        else:
            tangent = np.array([1.0, 0.0, 0.0])
        norm = float(np.linalg.norm(tangent))
        tangent = tangent / norm if norm > 1e-8 else np.array([1.0, 0.0, 0.0])
        normal = np.array([-tangent[1], tangent[0], 0.0])

        position_error = cur - closest_point
        lateral_error = float(np.dot(position_error, normal))
        longitudinal_error = float(np.dot(position_error, tangent))

        desired_yaw = math.atan2(float(tangent[1]), float(tangent[0]))
        orientation_error = math.atan2(
            math.sin(current_yaw - desired_yaw),
            math.cos(current_yaw - desired_yaw),
        )
        bin_size_deg = 10.0
        num_bins = int(360 / bin_size_deg)
        bin_index = int(((orientation_error + math.pi) * 180.0 / math.pi) / bin_size_deg) % num_bins
        discretized_orientation = (bin_index * bin_size_deg * math.pi / 180.0) - math.pi

        progress = idx / (len(path) - 1) if len(path) > 1 else 0.0
        deviation = float(np.linalg.norm(position_error))

        if velocity_2d is not None:
            speed_along_path = float(np.dot(np.asarray(velocity_2d, dtype=float), tangent[:2]))
        else:
            speed_along_path = 0.0

        box_forward_x = math.cos(current_yaw)
        box_forward_y = math.sin(current_yaw)

        return np.array(
            [
                lateral_error,
                longitudinal_error,
                discretized_orientation,
                float(progress),
                deviation,
                speed_along_path,
                box_forward_x,
                box_forward_y,
            ],
            dtype=np.float32,
        )
