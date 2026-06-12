#!/usr/bin/env python3

import math
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def _param_int(node: Node, name: str) -> int:
    value: Any = node.get_parameter(name).value
    return int(value)


def _param_float(node: Node, name: str) -> float:
    value: Any = node.get_parameter(name).value
    return float(value)


def _param_bool(node: Node, name: str) -> bool:
    value: Any = node.get_parameter(name).value
    return bool(value)


class HeightCell:
    __slots__ = ('samples', 'next_index', 'total_count')

    def __init__(self):
        self.samples = []
        self.next_index = 0
        self.total_count = 0

    def add(self, z, max_samples):
        self.total_count += 1
        if len(self.samples) < max_samples:
            self.samples.append(z)
            return

        self.samples[self.next_index] = z
        self.next_index = (self.next_index + 1) % max_samples

    def ground_height(self, lowest_fraction, min_points):
        if len(self.samples) < min_points:
            return None

        sorted_samples = sorted(self.samples)
        low_count = max(1, int(math.ceil(len(sorted_samples) * lowest_fraction)))
        low_samples = sorted_samples[:low_count]
        return sum(low_samples) / len(low_samples)

    def mse(self, ground_height):
        if not self.samples:
            return None

        error_sum = 0.0
        for z in self.samples:
            error = z - ground_height
            error_sum += error * error
        return error_sum / len(self.samples)


class GroundHeightGrid(Node):
    def __init__(self):
        super().__init__('ground_height_grid')

        self.declare_parameter('input_topic', '/fastlio2/world_cloud')
        self.declare_parameter('output_topic', '/ground_height_grid')
        self.declare_parameter('grid_resolution', 0.3)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('lowest_fraction', 0.2)
        self.declare_parameter('min_points_per_cell', 5)
        self.declare_parameter('max_samples_per_cell', 120)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('use_z_max_threshold', True)
        self.declare_parameter('z_max_threshold', -0.15)
        self.declare_parameter('max_cells', 200000)
        self.declare_parameter('use_bounds', False)
        self.declare_parameter('x_min', -50.0)
        self.declare_parameter('x_max', 50.0)
        self.declare_parameter('y_min', -50.0)
        self.declare_parameter('y_max', 50.0)
        self.declare_parameter('stats_log_period_sec', 10.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.grid_resolution = _param_float(self, 'grid_resolution')
        self.publish_rate_hz = _param_float(self, 'publish_rate_hz')
        self.lowest_fraction = _param_float(self, 'lowest_fraction')
        self.min_points_per_cell = _param_int(self, 'min_points_per_cell')
        self.max_samples_per_cell = _param_int(self, 'max_samples_per_cell')
        self.point_stride = _param_int(self, 'point_stride')
        self.use_z_max_threshold = _param_bool(self, 'use_z_max_threshold')
        self.z_max_threshold = _param_float(self, 'z_max_threshold')
        self.max_cells = _param_int(self, 'max_cells')
        self.use_bounds = _param_bool(self, 'use_bounds')
        self.x_min = _param_float(self, 'x_min')
        self.x_max = _param_float(self, 'x_max')
        self.y_min = _param_float(self, 'y_min')
        self.y_max = _param_float(self, 'y_max')
        self.stats_log_period_sec = _param_float(self, 'stats_log_period_sec')

        self.validate_parameters()

        self.lock = threading.Lock()
        self.cells = {}
        self.latest_header = None
        self.received_clouds = 0
        self.received_points = 0
        self.used_points = 0
        self.dropped_new_cells = 0

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_grid)
        self.stats_timer = self.create_timer(self.stats_log_period_sec, self.log_low_mean_stats)

        self.get_logger().info(
            f'Listening to {self.input_topic}; publishing ground height grid to '
            f'{self.output_topic} at {self.publish_rate_hz:.2f} Hz '
            f'with {self.grid_resolution:.2f} m cells'
        )

    def validate_parameters(self):
        if self.grid_resolution <= 0.0:
            raise ValueError('grid_resolution must be greater than 0.0')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than 0.0')
        if not 0.0 < self.lowest_fraction <= 1.0:
            raise ValueError('lowest_fraction must be in (0.0, 1.0]')
        if self.min_points_per_cell <= 0:
            raise ValueError('min_points_per_cell must be greater than 0')
        if self.max_samples_per_cell <= 0:
            raise ValueError('max_samples_per_cell must be greater than 0')
        if self.point_stride <= 0:
            raise ValueError('point_stride must be greater than 0')
        if self.max_cells <= 0:
            raise ValueError('max_cells must be greater than 0')
        if self.use_bounds and (self.x_min >= self.x_max or self.y_min >= self.y_max):
            raise ValueError('bounds require x_min < x_max and y_min < y_max')
        if self.stats_log_period_sec <= 0.0:
            raise ValueError('stats_log_period_sec must be greater than 0.0')

    def cloud_callback(self, msg):
        added_points = 0
        read_points = point_cloud2.read_points(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )

        with self.lock:
            self.latest_header = msg.header
            self.received_clouds += 1

            for point_index, point in enumerate(read_points):
                self.received_points += 1
                if point_index % self.point_stride != 0:
                    continue

                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                if self.use_z_max_threshold and z >= self.z_max_threshold:
                    continue
                if self.use_bounds and not self.in_bounds(x, y):
                    continue

                key = self.cell_key(x, y)
                cell = self.cells.get(key)
                if cell is None:
                    if len(self.cells) >= self.max_cells:
                        self.dropped_new_cells += 1
                        continue
                    cell = HeightCell()
                    self.cells[key] = cell

                cell.add(z, self.max_samples_per_cell)
                added_points += 1

            self.used_points += added_points

        if self.dropped_new_cells > 0:
            self.get_logger().warn(
                f'max_cells={self.max_cells} reached; dropped '
                f'{self.dropped_new_cells} new-cell samples',
                throttle_duration_sec=5.0,
            )

    def in_bounds(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def cell_key(self, x, y):
        return (
            math.floor(x / self.grid_resolution),
            math.floor(y / self.grid_resolution),
        )

    def cell_center(self, key):
        ix, iy = key
        return (
            (ix + 0.5) * self.grid_resolution,
            (iy + 0.5) * self.grid_resolution,
        )

    def publish_grid(self):
        with self.lock:
            if self.latest_header is None:
                return

            header = self.latest_header
            grid_points = []
            for key, cell in self.cells.items():
                height = cell.ground_height(
                    self.lowest_fraction,
                    self.min_points_per_cell,
                )
                if height is None:
                    continue

                mse = cell.mse(height)
                if mse is None:
                    continue

                x, y = self.cell_center(key)
                count = float(len(cell.samples))
                grid_points.append((x, y, height, height, mse, count))

        if not grid_points:
            return

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='mse', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='count', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, grid_points)
        self.publisher.publish(cloud)

    def log_low_mean_stats(self):
        with self.lock:
            low_mean_heights = []
            for cell in self.cells.values():
                height = cell.ground_height(
                    self.lowest_fraction,
                    self.min_points_per_cell,
                )
                if height is None:
                    continue

                low_mean_heights.append(height)

        if not low_mean_heights:
            self.get_logger().info(
                'Ground mean height: nan, mse: nan'
            )
            return

        mean_height = sum(low_mean_heights) / len(low_mean_heights)
        height_mse = sum(
            (height - mean_height) * (height - mean_height)
            for height in low_mean_heights
        ) / len(low_mean_heights)

        self.get_logger().info(
            f'Ground mean height: {mean_height:.6f}, mse: {height_mse:.6f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GroundHeightGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
