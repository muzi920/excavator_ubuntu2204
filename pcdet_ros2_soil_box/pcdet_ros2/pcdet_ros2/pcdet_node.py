"""ROS 2 wrapper for online OpenPCDet inference."""

from math import cos
from math import sin
from pathlib import Path

# from geometry_msgs.msg import PointStamped  # commented out: cmd_dig_point disabled
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

import numpy as np

from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network
from pcdet.models import load_data_to_gpu
from pcdet.utils import common_utils

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import torch

from vision_msgs.msg import Detection3D
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from .config import cfg
from .config import cfg_from_yaml_file
from .geometry import highest_point_in_oriented_box


class PCDetROS(Node):
    """Run OpenPCDet on incoming ``PointCloud2`` messages."""

    _FIELD_ALIASES = {
        'intensity': ('intensity', 'reflectivity', 'reflectance'),
        'timestamp': ('timestamp', 'time', 't', 'offset_time'),
        'elongation': ('elongation',),
    }

    def __init__(self):
        """Load parameters and the model, then create ROS interfaces."""
        super().__init__('pcdet')
        self._warned_missing_features = set()
        self._points = None
        # Temporal smoothing state for the Soil highest point.
        self._smoothed_highest = None
        self._highest_missing_frames = 0
        self._highest_point_score = 0.0
        self._highest_point_class = 'Soil'
        # Temporal smoothing state for detection boxes, keyed by class name.
        self._smoothed_boxes = {}
        self._init_parameters()
        self._load_config_and_model()
        self._init_interfaces()

    def _cloud_callback(self, cloud_msg):
        """Run detection for one point-cloud message and publish the result."""
        out_msg = Detection3DArray()
        out_msg.header = cloud_msg.header
        highest_points_msg = PoseArray()
        highest_points_msg.header = cloud_msg.header
        marker_array_msg = self._marker_delete_all(cloud_msg.header)
        box_highest_points_msg = PoseArray()
        box_highest_points_msg.header = cloud_msg.header
        box_marker_array_msg = self._marker_delete_all(cloud_msg.header)
        dig_xyz = None
        dump_xyz = None

        try:
            cloud_array = point_cloud2.read_points(
                cloud_msg,
                skip_nans=False,
            )
            points = self._convert_cloud_format(cloud_array)
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid input point cloud: {exc}')
            return

        scores, boxes_lidar, labels = self._run_inference(points)

        predictions = self._select_predictions(
            scores, boxes_lidar, labels
        )

        # Group this frame's detections by class (already <=1 per class).
        frame_boxes = {}
        frame_scores = {}
        for score, box_lidar, label_id in predictions:
            class_name = self._class_name(label_id)
            if (class_name not in frame_boxes
                    or float(score) > frame_scores[class_name]):
                frame_boxes[class_name] = box_lidar
                frame_scores[class_name] = float(score)

        smoothed_dets = self._resolve_detections(frame_boxes, frame_scores)

        for class_name in self._class_names:
            smoothed = smoothed_dets.get(class_name)
            if smoothed is None:
                continue
            box_lidar, box_score = smoothed

            detection = Detection3D()
            detection.header = cloud_msg.header

            quaternion = self._yaw_to_quaternion(float(box_lidar[6]))
            detection.bbox.center.orientation = quaternion
            detection.bbox.center.position.x = float(box_lidar[0])
            detection.bbox.center.position.y = float(box_lidar[1])
            detection.bbox.center.position.z = float(box_lidar[2])
            detection.bbox.size.x = float(box_lidar[3])
            detection.bbox.size.y = float(box_lidar[4])
            detection.bbox.size.z = float(box_lidar[5])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = float(box_score)
            hypothesis.pose.pose = detection.bbox.center
            detection.results.append(hypothesis)

            # Detection3D.id is reserved for a stable tracking ID. This node
            # performs detection only, so it must remain empty.
            detection.id = ''
            out_msg.detections.append(detection)

        # The Soil highest point is measured inside the *smoothed* Soil box,
        # then EMA-smoothed on top. When no Soil was detected this frame the
        # box is held and the highest point is held along with it.
        raw_highest_point = None
        highest_point_score = None
        highest_point_class = None
        for class_name in self._class_names:
            if not self._publish_highest_point_for(class_name):
                continue
            if class_name in frame_boxes and class_name in smoothed_dets:
                highest_point = highest_point_in_oriented_box(
                    points, smoothed_dets[class_name][0]
                )
                if highest_point is not None:
                    highest_point[2] += self._highest_point_z_offset
                    raw_highest_point = highest_point
                    highest_point_score = smoothed_dets[class_name][1]
                    highest_point_class = class_name
            break

        smoothed_highest = self._resolve_highest_point(
            raw_highest_point, highest_point_score, highest_point_class
        )
        if smoothed_highest is not None:
            dig_xyz = smoothed_highest
            pose = self._point_pose(smoothed_highest)
            highest_points_msg.poses.append(pose)
            self._append_highest_point_markers(
                marker_array_msg,
                cloud_msg.header,
                pose,
                self._highest_point_score,
                self._highest_point_class,
                0,
            )

            # dig_point = PointStamped()
            # dig_point.header = cloud_msg.header
            # dig_point.point.x = float(smoothed_highest[0])
            # dig_point.point.y = float(smoothed_highest[1])
            # dig_point.point.z = float(smoothed_highest[2])
            # self._cmd_dig_point_publisher.publish(dig_point)

        # The Box top-center point (cx, cy, cz + dz/2) is derived from the
        # smoothed Box, so it inherits the box smoothing for free.
        if 'Box' in smoothed_dets:
            box_lidar, box_score = smoothed_dets['Box']
            dump_xyz = self._box_top_center(box_lidar)
            box_top_pose = self._point_pose(dump_xyz)
            box_highest_points_msg.poses.append(box_top_pose)
            self._append_box_top_markers(
                box_marker_array_msg, cloud_msg.header, box_top_pose, box_score
            )

        # Publish empty arrays as well, so downstream consumers receive one
        # result for every successfully processed input frame.
        self._detection_publisher.publish(out_msg)
        self._highest_points_publisher.publish(highest_points_msg)
        self._highest_point_markers_publisher.publish(marker_array_msg)
        self._box_highest_points_publisher.publish(box_highest_points_msg)
        self._box_highest_point_markers_publisher.publish(box_marker_array_msg)

        # Full-cycle excavator command: dig = Soil highest point, dump = Box
        # top-center point. dump_z floor rule: z < 1.0 -> 1.0, else keep.
        if dig_xyz is not None and dump_xyz is not None:
            dump_z = float(dump_xyz[2])
            final_dump_z = dump_z
            if dump_z < 1.0:
                final_dump_z = 1.0
                self.get_logger().info(
                    '[FullCycle A] dump_z 规则判断: '
                    f'原始 z={dump_z:.3f}m  →  使用 z={final_dump_z:.3f}m  '
                    '(规则: z<1→1.0, z>=1→原值)',
                    throttle_duration_sec=2.0,
                )
            full_cycle_msg = Float64MultiArray()
            full_cycle_msg.layout.dim = [
                MultiArrayDimension(label='dig_dump', size=6, stride=6)
            ]
            full_cycle_msg.layout.data_offset = 0
            full_cycle_msg.data = [
                float(dig_xyz[0]), -float(dig_xyz[1]), float(dig_xyz[2]),
                float(dump_xyz[0]), -float(dump_xyz[1]), final_dump_z,
            ]
            self._cmd_full_cycle_publisher.publish(full_cycle_msg)

    def _select_predictions(self, scores, boxes_lidar, labels):
        """Apply thresholds and keep the highest-scoring predictions."""
        predictions = [
            (float(score), box_lidar, int(label))
            for score, box_lidar, label in zip(
                scores, boxes_lidar, labels
            )
            if self._allow_publishing(int(label), float(score))
        ]
        predictions.sort(key=lambda prediction: prediction[0], reverse=True)
        if self._max_detections_per_class > 0:
            class_counts = {}
            selected_predictions = []
            for prediction in predictions:
                label_id = prediction[2]
                count = class_counts.get(label_id, 0)
                if count >= self._max_detections_per_class:
                    continue
                selected_predictions.append(prediction)
                class_counts[label_id] = count + 1
            predictions = selected_predictions
        if self._max_detections_per_frame > 0:
            return predictions[:self._max_detections_per_frame]
        return predictions

    @staticmethod
    def _point_pose(point):
        """Create a valid pose whose position is the selected point."""
        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2])
        pose.orientation.w = 1.0
        return pose

    def _resolve_detections(self, frame_boxes, frame_scores):
        """Smooth and hold 3D boxes, one tracked state per class.

        Returns ``{class_name: (smoothed_box, score)}``. Classes detected in
        this frame are smoothed against their previous box; classes detected
        recently but missing from this frame are held for ``box_hold_frames``
        frames before being dropped from the output.
        """
        resolved = {}

        for class_name, raw_box in frame_boxes.items():
            raw = np.asarray(raw_box, dtype=np.float32)
            state = self._smoothed_boxes.get(class_name)
            if state is None:
                state = {
                    'box': raw.copy(),
                    'score': frame_scores[class_name],
                    'missing_frames': 0,
                }
                self._smoothed_boxes[class_name] = state
            else:
                state['missing_frames'] = 0
                state['score'] = frame_scores[class_name]
                state['box'] = self._smooth_box(state['box'], raw)
            resolved[class_name] = (state['box'].copy(), state['score'])

        for class_name, state in list(self._smoothed_boxes.items()):
            if class_name in frame_boxes:
                continue
            if state['missing_frames'] >= self._box_hold_frames:
                continue
            state['missing_frames'] += 1
            resolved[class_name] = (state['box'].copy(), state['score'])

        return resolved

    def _smooth_box(self, previous_box, raw_box):
        """EMA the box center/size and shortest-arc EMA the yaw, with jump-reset."""
        previous = np.asarray(previous_box, dtype=np.float32)
        raw = np.asarray(raw_box, dtype=np.float32)

        center_jump = float(np.linalg.norm(raw[:3] - previous[:3]))
        if center_jump > self._box_max_jump:
            # The target may have switched; re-anchor instead of smearing the
            # box across a large gap.
            return raw.copy()

        alpha = self._box_smoothing_alpha
        smoothed = previous.copy()
        smoothed[:6] = alpha * raw[:6] + (1.0 - alpha) * previous[:6]

        delta_yaw = float(raw[6] - previous[6])
        delta_yaw = (delta_yaw + np.pi) % (2.0 * np.pi) - np.pi
        smoothed[6] = np.float32(
            (float(previous[6]) + alpha * delta_yaw + np.pi) % (2.0 * np.pi)
            - np.pi
        )
        return smoothed

    def _resolve_highest_point(self, raw_highest_point, score, class_name):
        """Return this frame's Soil highest point after temporal smoothing.

        A fresh detection is smoothed against the previous value. When the
        detection is missing, the last smoothed point is held for
        ``highest_point_hold_frames`` frames and then dropped. Returns ``None``
        when there is nothing to publish this frame.
        """
        if raw_highest_point is not None:
            self._highest_missing_frames = 0
            self._highest_point_score = score
            self._highest_point_class = class_name
            return self._smooth_highest_point(raw_highest_point)

        if self._smoothed_highest is None:
            return None

        self._highest_missing_frames += 1
        if self._highest_missing_frames > self._highest_point_hold_frames:
            self._smoothed_highest = None
            return None
        return self._smoothed_highest.copy()

    def _smooth_highest_point(self, raw_point):
        """Apply EMA smoothing with a jump-reset to the Soil highest point."""
        raw = np.asarray(raw_point, dtype=np.float32)
        if self._smoothed_highest is None:
            self._smoothed_highest = raw.copy()
            return raw.copy()

        previous = self._smoothed_highest
        jump = float(np.linalg.norm(raw - previous))
        if jump > self._highest_point_max_jump:
            # The target may have switched or the box jumped; re-anchor on the
            # new position instead of dragging the smoothed point across.
            self._smoothed_highest = raw.copy()
            return raw.copy()

        alpha = self._highest_point_smoothing_alpha
        smoothed = alpha * raw + (1.0 - alpha) * previous
        self._smoothed_highest = smoothed
        return smoothed.copy()

    @staticmethod
    def _marker_delete_all(header):
        """Start a marker message by clearing markers from the last frame."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        return marker_array

    def _append_highest_point_markers(
        self, marker_array, header, pose, score, class_name, point_index
    ):
        """Append a sphere and coordinate label for one highest point."""
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'soil_highest_points'
        sphere.id = point_index * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose
        sphere.scale.x = self._highest_point_marker_scale
        sphere.scale.y = self._highest_point_marker_scale
        sphere.scale.z = self._highest_point_marker_scale
        sphere.color.r = 1.0
        sphere.color.g = 0.1
        sphere.color.b = 0.1
        sphere.color.a = 1.0
        marker_array.markers.append(sphere)

        text = Marker()
        text.header = header
        text.ns = 'soil_highest_point_labels'
        text.id = point_index * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = (
            pose.position.z + self._highest_point_text_offset
        )
        text.pose.orientation.w = 1.0
        text.scale.z = self._highest_point_text_height
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.1
        text.color.a = 1.0
        text.text = (
            f'{class_name} highest  score={score:.3f}\n'
            f'x={pose.position.x:.3f}  y={pose.position.y:.3f}  '
            f'z={pose.position.z:.3f} m'
        )
        marker_array.markers.append(text)

    def _publish_highest_point_for(self, class_name):
        """Return whether highest-point output is enabled for a class."""
        return (
            not self._highest_point_classes
            or class_name in self._highest_point_classes
        )

    @staticmethod
    def _box_top_center(box):
        """Return the center of a box's top face from [cx,cy,cz,dx,dy,dz,yaw]."""
        box = np.asarray(box, dtype=np.float32)
        return np.array(
            [box[0], box[1], box[2] + box[5] * 0.5], dtype=np.float32
        )

    def _append_box_top_markers(self, marker_array, header, pose, score):
        """Append a sphere and label marking the Box top-center point."""
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'box_highest_points'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose
        sphere.scale.x = self._highest_point_marker_scale
        sphere.scale.y = self._highest_point_marker_scale
        sphere.scale.z = self._highest_point_marker_scale
        sphere.color.r = 0.1
        sphere.color.g = 0.4
        sphere.color.b = 1.0
        sphere.color.a = 1.0
        marker_array.markers.append(sphere)

        text = Marker()
        text.header = header
        text.ns = 'box_highest_point_labels'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = (
            pose.position.z + self._highest_point_text_offset
        )
        text.pose.orientation.w = 1.0
        text.scale.z = self._highest_point_text_height
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.1
        text.color.a = 1.0
        text.text = (
            f'Box top  score={score:.3f}\n'
            f'x={pose.position.x:.3f}  y={pose.position.y:.3f}  '
            f'z={pose.position.z:.3f} m'
        )
        marker_array.markers.append(text)

    def _convert_cloud_format(self, cloud_array, remove_nans=True):
        """Convert a structured ROS cloud into the configured PCDet layout."""
        if cloud_array.dtype.names is None:
            raise TypeError('PointCloud2 data does not have named fields')

        available_fields = {
            name.lower(): name for name in cloud_array.dtype.names
        }
        missing_xyz = [
            name for name in ('x', 'y', 'z')
            if name not in available_fields
        ]
        if missing_xyz:
            raise ValueError(
                f'missing required fields {missing_xyz}; available fields are '
                f'{list(cloud_array.dtype.names)}'
            )

        # Flatten organized clouds before masking so the model always receives
        # an (N, C) array.
        cloud_array = cloud_array.reshape(-1)
        if remove_nans:
            mask = (
                np.isfinite(cloud_array[available_fields['x']])
                & np.isfinite(cloud_array[available_fields['y']])
                & np.isfinite(cloud_array[available_fields['z']])
            )
            cloud_array = cloud_array[mask]

        points = np.zeros(
            (cloud_array.shape[0], self._num_features),
            dtype=np.float32,
        )

        for index, feature_name in enumerate(self._feature_names):
            source_field = self._find_source_field(
                feature_name, available_fields
            )
            if source_field is None:
                self._warn_missing_feature(
                    feature_name, tuple(cloud_array.dtype.names)
                )
                continue

            values = np.asarray(
                cloud_array[source_field], dtype=np.float32
            )
            if values.ndim != 1:
                raise ValueError(
                    f'field {source_field!r} must contain one scalar per point'
                )

            if feature_name == 'intensity':
                values = values * self._intensity_scale
            elif feature_name == 'timestamp':
                values = values * self._timestamp_scale

            points[:, index] = values

        return np.ascontiguousarray(points)

    def _find_source_field(self, feature_name, available_fields):
        """Resolve an OpenPCDet feature name to an incoming ROS field."""
        normalized_name = feature_name.lower()
        candidates = self._FIELD_ALIASES.get(
            normalized_name, (normalized_name,)
        )
        for candidate in candidates:
            if candidate in available_fields:
                return available_fields[candidate]
        return None

    def _warn_missing_feature(self, feature_name, available_fields):
        """Warn once when an optional model feature is absent."""
        if feature_name in self._warned_missing_features:
            return
        self.get_logger().warning(
            f'PointCloud2 has no field for model feature {feature_name!r}; '
            f'using zeros. Available fields: {list(available_fields)}'
        )
        self._warned_missing_features.add(feature_name)

    def _run_inference(self, points):
        """Prepare one point cloud and run the network."""
        if points.shape[0] == 0:
            return (
                np.empty((0,), dtype=np.float32),
                np.empty((0, 7), dtype=np.float32),
                np.empty((0,), dtype=np.int64),
            )

        self._points = points.reshape((-1, self._num_features))
        input_dict = {'points': self._points}

        with torch.inference_mode():
            data_dict = self._online_detection.prepare_data(
                data_dict=input_dict
            )
            data_dict = self._online_detection.collate_batch([data_dict])
            load_data_to_gpu(data_dict)
            prediction_dicts, _ = self._network(data_dict)

            prediction = prediction_dicts[0]
            boxes_lidar = prediction['pred_boxes'].detach().cpu().numpy()
            scores = prediction['pred_scores'].detach().cpu().numpy()
            labels = prediction['pred_labels'].detach().cpu().numpy()

        return scores, boxes_lidar, labels

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Convert a LiDAR-frame yaw angle into a quaternion."""
        quaternion = Quaternion()
        half_yaw = yaw * 0.5
        quaternion.z = sin(half_yaw)
        quaternion.w = cos(half_yaw)
        return quaternion

    def _allow_publishing(self, label_id, score):
        """Apply the optional per-class output threshold."""
        if not self._allow_score_thresholding:
            return True

        threshold_index = label_id - 1
        if 0 <= threshold_index < len(self._threshold_array):
            return score >= self._threshold_array[threshold_index]
        return True

    def _class_name(self, label_id):
        """Return the configured class name for a one-based PCDet label."""
        class_index = label_id - 1
        if 0 <= class_index < len(self._class_names):
            return self._class_names[class_index]
        self.get_logger().warning(
            f'Model returned unknown class label {label_id}'
        )
        return str(label_id)

    def _load_config_and_model(self):
        """Load the OpenPCDet configuration, checkpoint and CUDA model."""
        if not self._config_file.is_file():
            raise FileNotFoundError(
                f'OpenPCDet config not found: {self._config_file}'
            )
        if not self._model_file.is_file():
            raise FileNotFoundError(
                f'OpenPCDet checkpoint not found: {self._model_file}'
            )

        cfg_from_yaml_file(
            str(self._config_file), cfg, str(self._package_folder_path)
        )
        cfg.DATA_CONFIG._BASE_CONFIG_ = str(
            self._package_folder_path / cfg.DATA_CONFIG._BASE_CONFIG_
        )

        self._class_names = list(cfg.CLASS_NAMES)
        unknown_highest_point_classes = (
            self._highest_point_classes - set(self._class_names)
        )
        if unknown_highest_point_classes:
            raise ValueError(
                'highest_point_classes contains unknown classes: '
                f'{sorted(unknown_highest_point_classes)}'
            )
        self._feature_names = list(
            cfg.DATA_CONFIG.POINT_FEATURE_ENCODING.src_feature_list
        )
        if self._num_features != len(self._feature_names):
            raise ValueError(
                f'num_features={self._num_features}, but the model expects '
                f'{len(self._feature_names)} features: {self._feature_names}'
            )

        if not torch.cuda.is_available():
            raise RuntimeError(
                'CUDA is required for real-time OpenPCDet inference, but '
                'torch.cuda.is_available() is False'
            )
        if not 0 <= self._device_id < torch.cuda.device_count():
            raise ValueError(
                f'device_id={self._device_id} is invalid; detected '
                f'{torch.cuda.device_count()} CUDA device(s)'
            )

        torch.cuda.set_device(self._device_id)
        self._device = torch.device(f'cuda:{self._device_id}')

        if self._allow_memory_fractioning:
            if not 0.0 < self._device_memory_fraction <= 1.0:
                raise ValueError(
                    'device_memory_fraction must be in the range (0, 1]'
                )
            torch.cuda.set_per_process_memory_fraction(
                self._device_memory_fraction,
                device=self._device_id,
            )

        self._pcdet_logger = common_utils.create_logger()
        self._online_detection = DatasetTemplate(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            root_path=self._package_folder_path,
            logger=self._pcdet_logger,
        )
        self._network = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(cfg.CLASS_NAMES),
            dataset=self._online_detection,
        )
        self._network.load_params_from_file(
            filename=str(self._model_file),
            logger=self._pcdet_logger,
            to_cpu=True,
        )
        self._network = self._network.to(self._device).eval()

        self.get_logger().info(
            f'Loaded {cfg.MODEL.NAME} on '
            f'{torch.cuda.get_device_name(self._device_id)}; '
            f'classes={self._class_names}, features={self._feature_names}'
        )

    def _init_parameters(self):
        """Declare and read ROS parameters."""
        self.declare_parameter('config_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter(
            'package_folder_path', rclpy.Parameter.Type.STRING
        )
        self.declare_parameter('model_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter(
            'allow_memory_fractioning', rclpy.Parameter.Type.BOOL
        )
        self.declare_parameter(
            'allow_score_thresholding', rclpy.Parameter.Type.BOOL
        )
        self.declare_parameter('num_features', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('device_id', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter(
            'device_memory_fraction', rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            'threshold_array', rclpy.Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter('intensity_scale', 1.0)
        self.declare_parameter('timestamp_scale', 1.0)
        self.declare_parameter('max_detections_per_frame', 0)
        self.declare_parameter('max_detections_per_class', 0)
        self.declare_parameter(
            'highest_point_classes', rclpy.Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter('highest_point_marker_scale', 0.15)
        self.declare_parameter('highest_point_text_height', 0.15)
        self.declare_parameter('highest_point_text_offset', 0.25)
        self.declare_parameter('highest_point_smoothing_alpha', 0.35)
        self.declare_parameter('highest_point_hold_frames', 5)
        self.declare_parameter('highest_point_max_jump', 2.0)
        self.declare_parameter('highest_point_z_offset', -0.1)
        self.declare_parameter('box_smoothing_alpha', 0.5)
        self.declare_parameter('box_hold_frames', 5)
        self.declare_parameter('box_max_jump', 2.0)

        package_folder = self.get_parameter('package_folder_path').value
        self._package_folder_path = Path(package_folder)
        self._config_file = (
            self._package_folder_path
            / self.get_parameter('config_file').value
        )
        self._model_file = (
            self._package_folder_path
            / self.get_parameter('model_file').value
        )
        self._allow_memory_fractioning = self.get_parameter(
            'allow_memory_fractioning'
        ).value
        self._allow_score_thresholding = self.get_parameter(
            'allow_score_thresholding'
        ).value
        self._num_features = self.get_parameter('num_features').value
        self._device_id = self.get_parameter('device_id').value
        self._device_memory_fraction = self.get_parameter(
            'device_memory_fraction'
        ).value
        self._threshold_array = list(
            self.get_parameter('threshold_array').value
        )
        self._intensity_scale = self.get_parameter(
            'intensity_scale'
        ).value
        self._timestamp_scale = self.get_parameter(
            'timestamp_scale'
        ).value
        self._max_detections_per_frame = self.get_parameter(
            'max_detections_per_frame'
        ).value
        self._max_detections_per_class = self.get_parameter(
            'max_detections_per_class'
        ).value
        self._highest_point_classes = set(
            self.get_parameter('highest_point_classes').value
        )
        self._highest_point_marker_scale = self.get_parameter(
            'highest_point_marker_scale'
        ).value
        self._highest_point_text_height = self.get_parameter(
            'highest_point_text_height'
        ).value
        self._highest_point_text_offset = self.get_parameter(
            'highest_point_text_offset'
        ).value
        self._highest_point_smoothing_alpha = self.get_parameter(
            'highest_point_smoothing_alpha'
        ).value
        self._highest_point_hold_frames = self.get_parameter(
            'highest_point_hold_frames'
        ).value
        self._highest_point_max_jump = self.get_parameter(
            'highest_point_max_jump'
        ).value
        self._highest_point_z_offset = self.get_parameter(
            'highest_point_z_offset'
        ).value
        self._box_smoothing_alpha = self.get_parameter(
            'box_smoothing_alpha'
        ).value
        self._box_hold_frames = self.get_parameter(
            'box_hold_frames'
        ).value
        self._box_max_jump = self.get_parameter(
            'box_max_jump'
        ).value

        for parameter_name, value in (
            ('highest_point_marker_scale', self._highest_point_marker_scale),
            ('highest_point_text_height', self._highest_point_text_height),
        ):
            if value <= 0.0:
                raise ValueError(f'{parameter_name} must be greater than 0')
        if self._max_detections_per_frame < 0:
            raise ValueError(
                'max_detections_per_frame must be zero or greater'
            )
        if self._max_detections_per_class < 0:
            raise ValueError(
                'max_detections_per_class must be zero or greater'
            )
        if not 0.0 <= self._highest_point_smoothing_alpha <= 1.0:
            raise ValueError('highest_point_smoothing_alpha must be in [0, 1]')
        if self._highest_point_hold_frames < 0:
            raise ValueError(
                'highest_point_hold_frames must be zero or greater'
            )
        if self._highest_point_max_jump < 0.0:
            raise ValueError(
                'highest_point_max_jump must be zero or greater'
            )
        if not 0.0 <= self._box_smoothing_alpha <= 1.0:
            raise ValueError('box_smoothing_alpha must be in [0, 1]')
        if self._box_hold_frames < 0:
            raise ValueError('box_hold_frames must be zero or greater')
        if self._box_max_jump < 0.0:
            raise ValueError('box_max_jump must be zero or greater')

    def _init_interfaces(self):
        """Create a latest-frame sensor subscription and result publisher."""
        point_cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            'input',
            self._cloud_callback,
            point_cloud_qos,
        )
        self._detection_publisher = self.create_publisher(
            Detection3DArray,
            'output',
            10,
        )
        self._highest_points_publisher = self.create_publisher(
            PoseArray,
            'highest_points',
            10,
        )
        self._highest_point_markers_publisher = self.create_publisher(
            MarkerArray,
            'highest_point_markers',
            10,
        )
        self._box_highest_points_publisher = self.create_publisher(
            PoseArray,
            'box_highest_points',
            10,
        )
        self._box_highest_point_markers_publisher = self.create_publisher(
            MarkerArray,
            'box_highest_point_markers',
            10,
        )
        # self._cmd_dig_point_publisher = self.create_publisher(
        #     PointStamped,
        #     'cmd_dig_point',
        #     10,
        # )
        self._cmd_full_cycle_publisher = self.create_publisher(
            Float64MultiArray,
            'cmd_full_cycle',
            10,
        )


def main(args=None):
    """Run the OpenPCDet ROS 2 node."""
    rclpy.init(args=args)
    pcdet_node = PCDetROS()
    try:
        rclpy.spin(pcdet_node)
    except KeyboardInterrupt:
        pass
    finally:
        pcdet_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
