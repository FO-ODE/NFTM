#!/usr/bin/env python3

import math
import os
import signal
import threading
from typing import Any

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from numpy.typing import NDArray

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _param_int(node: Node, name: str) -> int:
    value: Any = node.get_parameter(name).value
    return int(value)


def _param_float(node: Node, name: str) -> float:
    value: Any = node.get_parameter(name).value
    return float(value)


def _param_bool(node: Node, name: str) -> bool:
    value: Any = node.get_parameter(name).value
    return bool(value)


def _param_str(node: Node, name: str) -> str:
    value: Any = node.get_parameter(name).value
    return str(value)


def quaternion_to_rotation_matrix(q) -> NDArray[np.float64]:
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_to_matrix(transform) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    rotation = quaternion_to_rotation_matrix(transform.rotation)
    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)
    return rotation, translation


def pose_to_matrix(pose) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    rotation = quaternion_to_rotation_matrix(pose.orientation)
    translation = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ], dtype=np.float64)
    return rotation, translation


class EgoCentricMap(Node):
    def __init__(self):
        super().__init__('ego_centric_map')

        self.declare_parameter('input_topic', '/fastlio2/world_cloud')
        self.declare_parameter('odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('array_topic', '/ego_centric_map')
        self.declare_parameter('marker_topic', '/ego_centric_map_markers')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('grid_cols', 17)
        self.declare_parameter('grid_rows', 11)
        self.declare_parameter('scan_size_x', 1.6)
        self.declare_parameter('scan_size_y', 1.0)
        self.declare_parameter('center_patch_size', 0.01)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('plot_update_interval_ms', 100)
        self.declare_parameter('color_min', 0.0)
        self.declare_parameter('color_max', 0.8)

        self.input_topic = _param_str(self, 'input_topic')
        self.odom_topic = _param_str(self, 'odom_topic')
        self.array_topic = _param_str(self, 'array_topic')
        self.marker_topic = _param_str(self, 'marker_topic')
        self.base_frame = _param_str(self, 'base_frame')
        self.grid_cols = _param_int(self, 'grid_cols')
        self.grid_rows = _param_int(self, 'grid_rows')
        self.scan_size_x = _param_float(self, 'scan_size_x')
        self.scan_size_y = _param_float(self, 'scan_size_y')
        self.center_patch_size = _param_float(self, 'center_patch_size')
        self.point_stride = _param_int(self, 'point_stride')
        self.publish_rate_hz = _param_float(self, 'publish_rate_hz')
        self.plot_update_interval_ms = _param_int(self, 'plot_update_interval_ms')
        self.color_min = _param_float(self, 'color_min')
        self.color_max = _param_float(self, 'color_max')

        self.validate_parameters()

        self.x_centers = np.linspace(-self.scan_size_x / 2.0, self.scan_size_x / 2.0, self.grid_cols)
        self.y_centers = np.linspace(-self.scan_size_y / 2.0, self.scan_size_y / 2.0, self.grid_rows)
        self.cell_step_x = self.x_centers[1] - self.x_centers[0] if self.grid_cols > 1 else self.scan_size_x
        self.cell_step_y = self.y_centers[1] - self.y_centers[0] if self.grid_rows > 1 else self.scan_size_y
        self.patch_half = self.center_patch_size / 2.0
        self.max_abs_x = self.scan_size_x / 2.0 + self.patch_half
        self.max_abs_y = self.scan_size_y / 2.0 + self.patch_half

        self.lock = threading.Lock()
        self.latest_grid = np.full((self.grid_rows, self.grid_cols), np.nan, dtype=np.float32)
        self.latest_markers = []
        self.latest_header = None
        self.latest_odom = None
        self.last_marker_count = 0
        self.received_clouds = 0
        self.used_clouds = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.array_pub = self.create_publisher(Float32MultiArray, self.array_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_outputs)

        self.get_logger().info(
            f'Listening to {self.input_topic}; grid {self.grid_cols}x{self.grid_rows} '
            f'over {self.scan_size_x:.2f}m x {self.scan_size_y:.2f}m in {self.base_frame}'
        )

    def validate_parameters(self):
        if self.grid_cols <= 0 or self.grid_rows <= 0:
            raise ValueError('grid_cols and grid_rows must be greater than 0')
        if self.scan_size_x <= 0.0 or self.scan_size_y <= 0.0:
            raise ValueError('scan_size_x and scan_size_y must be greater than 0.0')
        if self.center_patch_size <= 0.0:
            raise ValueError('center_patch_size must be greater than 0.0')
        if self.point_stride <= 0:
            raise ValueError('point_stride must be greater than 0')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than 0.0')
        if self.plot_update_interval_ms <= 0:
            raise ValueError('plot_update_interval_ms must be greater than 0')
        if self.color_min >= self.color_max:
            raise ValueError('color_min must be less than color_max')

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg

    def cloud_callback(self, msg):
        cloud_frame = msg.header.frame_id or 'world'
        pose_data = self.lookup_base_pose(cloud_frame)
        if pose_data is None:
            return

        rotation_cb, translation_cb, base_world_z = pose_data

        max_world_z = np.full((self.grid_rows, self.grid_cols), -np.inf, dtype=np.float64)
        read_points = point_cloud2.read_points(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )

        used_points = 0
        for point_index, point in enumerate(read_points):
            if point_index % self.point_stride != 0:
                continue

            p_world = np.array([float(point[0]), float(point[1]), float(point[2])], dtype=np.float64)
            p_base = rotation_cb @ p_world + translation_cb
            base_x = p_base[0]
            base_y = p_base[1]
            if abs(base_x) > self.max_abs_x or abs(base_y) > self.max_abs_y:
                continue

            col = int(round((base_x + self.scan_size_x / 2.0) / self.cell_step_x)) if self.grid_cols > 1 else 0
            row = int(round((base_y + self.scan_size_y / 2.0) / self.cell_step_y)) if self.grid_rows > 1 else 0
            if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
                continue
            if abs(base_x - self.x_centers[col]) > self.patch_half:
                continue
            if abs(base_y - self.y_centers[row]) > self.patch_half:
                continue

            max_world_z[row, col] = max(max_world_z[row, col], p_world[2])
            used_points += 1

        clearance_grid = np.full((self.grid_rows, self.grid_cols), np.nan, dtype=np.float32)
        valid_mask = np.isfinite(max_world_z)
        clearance_grid[valid_mask] = (base_world_z - max_world_z[valid_mask]).astype(np.float32)

        markers = []
        marker_id = 0
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                if not valid_mask[row, col]:
                    continue
                clearance = float(clearance_grid[row, col])
                z_base = float(max_world_z[row, col] - base_world_z)
                marker = self.make_cell_marker(
                    marker_id,
                    float(self.x_centers[col]),
                    float(self.y_centers[row]),
                    z_base,
                    clearance,
                )
                markers.append(marker)
                marker_id += 1

        visual_header = Header()
        visual_header.stamp = msg.header.stamp
        visual_header.frame_id = self.base_frame

        with self.lock:
            self.latest_grid = clearance_grid
            self.latest_markers = markers
            self.latest_header = visual_header
            self.received_clouds += 1
            if used_points > 0:
                self.used_clouds += 1

    def lookup_base_pose(self, cloud_frame):
        try:
            cloud_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                cloud_frame,
                Time(),
            )
            base_to_cloud = self.tf_buffer.lookup_transform(
                cloud_frame,
                self.base_frame,
                Time(),
            )
        except Exception as exc:
            odom_pose = self.lookup_base_pose_from_odom(cloud_frame)
            if odom_pose is not None:
                self.get_logger().warn(
                    f'Using odom fallback because TF lookup between {cloud_frame} '
                    f'and {self.base_frame} failed: {exc}',
                    throttle_duration_sec=5.0,
                )
                return odom_pose

            self.get_logger().warn(
                f'Cannot lookup TF between {cloud_frame} and {self.base_frame}, '
                f'and no usable odom fallback is available: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        rotation_cb, translation_cb = transform_to_matrix(cloud_to_base.transform)
        _, translation_bc = transform_to_matrix(base_to_cloud.transform)
        return rotation_cb, translation_cb, float(translation_bc[2])

    def lookup_base_pose_from_odom(self, cloud_frame):
        with self.lock:
            odom = self.latest_odom

        if odom is None:
            return None
        if odom.header.frame_id and odom.header.frame_id != cloud_frame:
            self.get_logger().warn(
                f'Odom fallback frame mismatch: odom frame is {odom.header.frame_id}, '
                f'cloud frame is {cloud_frame}',
                throttle_duration_sec=5.0,
            )
            return None
        odom_child_frame = odom.child_frame_id or self.base_frame
        rotation_child_to_cloud, translation_child_to_cloud = pose_to_matrix(odom.pose.pose)
        if odom_child_frame == self.base_frame:
            rotation_base_to_cloud = rotation_child_to_cloud
            translation_base_to_cloud = translation_child_to_cloud
        else:
            try:
                base_to_child = self.tf_buffer.lookup_transform(
                    odom_child_frame,
                    self.base_frame,
                    Time(),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'Odom fallback has child frame {odom_child_frame}, but cannot '
                    f'lookup local TF from {self.base_frame} to {odom_child_frame}: {exc}',
                    throttle_duration_sec=5.0,
                )
                return None

            rotation_base_to_child, translation_base_to_child = transform_to_matrix(base_to_child.transform)
            rotation_base_to_cloud = rotation_child_to_cloud @ rotation_base_to_child
            translation_base_to_cloud = (
                rotation_child_to_cloud @ translation_base_to_child
                + translation_child_to_cloud
            )

        rotation_cloud_to_base = rotation_base_to_cloud.transpose()
        translation_cloud_to_base = -rotation_cloud_to_base @ translation_base_to_cloud
        return (
            rotation_cloud_to_base,
            translation_cloud_to_base,
            float(translation_base_to_cloud[2]),
        )

    def make_cell_marker(self, marker_id, x, y, z, clearance):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = 'ego_centric_map'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.01, self.cell_step_x * 0.85)
        marker.scale.y = max(0.01, self.cell_step_y * 0.85)
        marker.scale.z = 0.01
        marker.color.r, marker.color.g, marker.color.b = self.color_for_value(clearance)
        marker.color.a = 0.85
        return marker

    def color_for_value(self, value):
        ratio = (value - self.color_min) / (self.color_max - self.color_min)
        ratio = min(1.0, max(0.0, ratio))
        rgba = cm.get_cmap('viridis')(ratio)
        return float(rgba[0]), float(rgba[1]), float(rgba[2])

    def publish_outputs(self):
        with self.lock:
            if self.latest_header is None:
                return
            grid = np.array(self.latest_grid, copy=True)
            markers = list(self.latest_markers)
            header = self.latest_header

        array_msg = Float32MultiArray()
        array_msg.layout.dim = [
            MultiArrayDimension(label='rows_y', size=self.grid_rows, stride=self.grid_rows * self.grid_cols),
            MultiArrayDimension(label='cols_x', size=self.grid_cols, stride=self.grid_cols),
        ]
        array_msg.layout.data_offset = 0
        array_msg.data = grid.reshape(-1).astype(np.float32).tolist()
        self.array_pub.publish(array_msg)

        for marker in markers:
            marker.header = header

        valid_marker_count = len(markers)
        for marker_id in range(valid_marker_count, self.last_marker_count):
            marker = Marker()
            marker.header = header
            marker.ns = 'ego_centric_map'
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.append(marker)

        self.last_marker_count = valid_marker_count
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    def snapshot(self):
        with self.lock:
            return np.array(self.latest_grid, copy=True)


class EgoCentricMapPlot:
    def __init__(self, node):
        self.node = node
        self.figure, self.axis = plt.subplots(figsize=(10, 6))
        self.image = self.axis.imshow(
            np.full((self.node.grid_rows, self.node.grid_cols), np.nan),
            origin='lower',
            cmap='viridis',
            vmin=self.node.color_min,
            vmax=self.node.color_max,
            extent=[
                -self.node.scan_size_x / 2.0,
                self.node.scan_size_x / 2.0,
                -self.node.scan_size_y / 2.0,
                self.node.scan_size_y / 2.0,
            ],
            aspect='equal',
        )
        self.texts = []
        for row, y in enumerate(self.node.y_centers):
            row_texts = []
            for col, x in enumerate(self.node.x_centers):
                text = self.axis.text(
                    x,
                    y,
                    '',
                    ha='center',
                    va='center',
                    color='white',
                    fontsize=7,
                )
                row_texts.append(text)
            self.texts.append(row_texts)

        self.axis.set_xlabel(f'{self.node.base_frame} x (m)')
        self.axis.set_ylabel(f'{self.node.base_frame} y (m)')
        self.axis.set_title('Base Ground Clearance Grid (m)')
        self.axis.set_xticks(self.node.x_centers)
        self.axis.set_yticks(self.node.y_centers)
        self.axis.grid(color='black', linewidth=0.4, alpha=0.25)
        self.colorbar = self.figure.colorbar(self.image, ax=self.axis)
        self.colorbar.set_label('base z - highest point z (m)')
        self.figure.tight_layout()

    def update(self, _frame):
        grid = self.node.snapshot()
        self.image.set_data(grid)
        for row in range(self.node.grid_rows):
            for col in range(self.node.grid_cols):
                value = grid[row, col]
                self.texts[row][col].set_text('' if np.isnan(value) else f'{value:.2f}')
        return [self.image, *[text for row in self.texts for text in row]]


def main(args=None):
    rclpy.init(args=args)
    node = EgoCentricMap()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    plot = EgoCentricMapPlot(node)
    animation = FuncAnimation(
        plot.figure,
        plot.update,
        interval=node.plot_update_interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    def handle_sigint(_signum, _frame):
        plt.close(plot.figure)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        plt.show()
    finally:
        event_source = getattr(animation, 'event_source', None)
        if event_source is not None:
            event_source.stop()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
