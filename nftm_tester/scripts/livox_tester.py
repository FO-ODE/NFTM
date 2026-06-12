#!/usr/bin/env python3

import math
import signal
import threading
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError as exc:
    CustomMsg = None
    CUSTOM_MSG_IMPORT_ERROR = exc
else:
    CUSTOM_MSG_IMPORT_ERROR = None


AXES = ('x', 'y', 'z')
DEFAULT_LINE_IDS = (0, 1, 2, 3)
LINE_COLORS = {
    0: '#d62728',
    1: '#2ca02c',
    2: '#1f77b4',
    3: '#9467bd',
}
LINE_SHOW_PARAMS = {
    0: 'show_line0',
    1: 'show_line1',
    2: 'show_line2',
    3: 'show_line3',
}


class LivoxTester(Node):
    def __init__(self):
        super().__init__('livox_tester')

        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('time_axis', 'ros')
        self.declare_parameter('line_ids', list(DEFAULT_LINE_IDS))
        self.declare_parameter('show_x', True)
        self.declare_parameter('show_y', True)
        self.declare_parameter('show_z', True)
        self.declare_parameter('show_imu', True)
        self.declare_parameter('show_line0', True)
        self.declare_parameter('show_line1', True)
        self.declare_parameter('show_line2', True)
        self.declare_parameter('show_line3', True)
        self.declare_parameter('window_s', 0.1)
        self.declare_parameter('update_interval_ms', 100)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('draw_points', True)
        self.declare_parameter('connect_points', False)
        self.declare_parameter('sort_points', True)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.time_axis = str(self.get_parameter('time_axis').value).lower()
        self.enabled_axes = tuple(
            axis
            for axis in AXES
            if bool(self.get_parameter(f'show_{axis}').value)
        )
        self.show_imu = bool(self.get_parameter('show_imu').value)
        line_enabled = {
            line_id: bool(self.get_parameter(parameter_name).value)
            for line_id, parameter_name in LINE_SHOW_PARAMS.items()
        }
        self.line_ids = self.filter_line_ids(
            self.get_parameter('line_ids').value,
            line_enabled,
        )
        self.window_s = float(self.get_parameter('window_s').value)
        self.update_interval_ms = int(self.get_parameter('update_interval_ms').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.point_stride = int(self.get_parameter('point_stride').value)
        self.draw_points = bool(self.get_parameter('draw_points').value)
        self.connect_points = bool(self.get_parameter('connect_points').value)
        self.sort_points = bool(self.get_parameter('sort_points').value)

        if not self.enabled_axes and not self.show_imu:
            raise ValueError('Enable at least one plot with show_x/show_y/show_z/show_imu')
        if self.enabled_axes and CustomMsg is None:
            raise RuntimeError(
                'Cannot import livox_ros_driver2.msg.CustomMsg. '
                'Source the workspace that contains livox_ros_driver2.'
            ) from CUSTOM_MSG_IMPORT_ERROR
        if self.enabled_axes and not self.line_ids:
            raise ValueError('At least one configured line must be enabled')
        if any(line_id < 0 for line_id in self.line_ids):
            raise ValueError('line_ids must be greater than or equal to 0')
        if self.window_s <= 0.0:
            raise ValueError('window_s must be greater than 0.0')
        if self.update_interval_ms <= 0:
            raise ValueError('update_interval_ms must be greater than 0')
        if self.max_points <= 0:
            raise ValueError('max_points must be greater than 0')
        if self.point_stride <= 0:
            raise ValueError('point_stride must be greater than 0')
        if self.time_axis not in ('ros', 'lidar'):
            raise ValueError('time_axis must be either "ros" or "lidar"')

        self.lock = threading.Lock()
        self.lidar_start_time = None
        self.ros_start_time = None
        self.times = {
            line_id: deque()
            for line_id in self.line_ids
        }
        self.values = {
            line_id: {
                axis: deque()
                for axis in self.enabled_axes
            }
            for line_id in self.line_ids
        }
        self.imu_times = deque()
        self.imu_accel_magnitudes = deque()
        self.latest_imu_time = None
        self.latest_imu_accel_magnitude = None
        self.total_messages = 0
        self.total_line_points = {line_id: 0 for line_id in self.line_ids}
        self.total_plotted_points = {line_id: 0 for line_id in self.line_ids}
        self.selected_line_ids = set(self.line_ids)

        self.imu_subscription = None
        self.lidar_subscription = None

        if self.show_imu:
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos_profile_sensor_data,
            )
        if self.enabled_axes:
            self.lidar_subscription = self.create_subscription(
                CustomMsg,
                self.lidar_topic,
                self.lidar_callback,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f'Listening to {self.lidar_topic} and {self.imu_topic}; '
            f'plotting axes {self.enabled_axes}, lines {self.line_ids}, show_imu={self.show_imu} '
            f'over a {self.window_s:.2f}s window with time_axis={self.time_axis} '
            f'with max_points={self.max_points}, point_stride={self.point_stride}'
        )

    @staticmethod
    def filter_line_ids(configured_line_ids, line_enabled):
        line_ids = []
        seen = set()
        for configured_line_id in configured_line_ids:
            line_id = int(configured_line_id)
            if line_id in seen:
                continue
            if not line_enabled.get(line_id, True):
                continue
            line_ids.append(line_id)
            seen.add(line_id)
        return tuple(line_ids)

    def point_time(self, msg, point, ros_receive_time, max_offset_time):
        if self.time_axis == 'lidar':
            return self.relative_lidar_time((int(msg.timebase) + int(point.offset_time)) * 1e-9)

        return ros_receive_time + (int(point.offset_time) - max_offset_time) * 1e-9

    def imu_callback(self, msg):
        accel = msg.linear_acceleration
        accel_magnitude = math.sqrt(
            float(accel.x) * float(accel.x)
            + float(accel.y) * float(accel.y)
            + float(accel.z) * float(accel.z)
        )
        now = self.get_clock().now().nanoseconds * 1e-9

        with self.lock:
            self.latest_imu_time = self.relative_ros_time(now)
            self.latest_imu_accel_magnitude = accel_magnitude
            self.imu_times.append(self.latest_imu_time)
            self.imu_accel_magnitudes.append(accel_magnitude)
            self.trim_imu_locked(self.latest_imu_time - self.window_s)

    def lidar_callback(self, msg):
        line_point_counts = {line_id: 0 for line_id in self.line_ids}
        samples = {line_id: [] for line_id in self.line_ids}
        ros_receive_time = self.relative_ros_time(self.get_clock().now().nanoseconds * 1e-9)
        max_offset_time = max((int(point.offset_time) for point in msg.points), default=0)
        for point in msg.points:
            line_id = int(point.line)
            if line_id not in self.selected_line_ids:
                continue

            line_point_counts[line_id] += 1
            if (line_point_counts[line_id] - 1) % self.point_stride != 0:
                continue

            samples[line_id].append(
                (
                    self.point_time(msg, point, ros_receive_time, max_offset_time),
                    float(point.x),
                    float(point.y),
                    float(point.z),
                )
            )

        if not any(line_point_counts.values()):
            with self.lock:
                self.total_messages += 1
            return

        if self.sort_points:
            for line_samples in samples.values():
                line_samples.sort(key=lambda sample: sample[0])

        with self.lock:
            latest_time = None
            for line_id, line_samples in samples.items():
                for sample_time, x_value, y_value, z_value in line_samples:
                    self.times[line_id].append(sample_time)
                    if 'x' in self.enabled_axes:
                        self.values[line_id]['x'].append(x_value)
                    if 'y' in self.enabled_axes:
                        self.values[line_id]['y'].append(y_value)
                    if 'z' in self.enabled_axes:
                        self.values[line_id]['z'].append(z_value)
                    latest_time = sample_time if latest_time is None else max(latest_time, sample_time)

            self.total_messages += 1
            for line_id in self.line_ids:
                self.total_line_points[line_id] += line_point_counts[line_id]
                self.total_plotted_points[line_id] += len(samples[line_id])
            if latest_time is not None:
                self.trim_locked(latest_time - self.window_s)

    def relative_lidar_time(self, absolute_time):
        if self.lidar_start_time is None:
            self.lidar_start_time = absolute_time
        return absolute_time - self.lidar_start_time

    def relative_ros_time(self, absolute_time):
        if self.ros_start_time is None:
            self.ros_start_time = absolute_time
        return absolute_time - self.ros_start_time

    def trim_locked(self, cutoff):
        for line_id in self.line_ids:
            while self.times[line_id] and self.times[line_id][0] < cutoff:
                self.times[line_id].popleft()
                for values in self.values[line_id].values():
                    values.popleft()

    def trim_imu_locked(self, cutoff):
        while self.imu_times and self.imu_times[0] < cutoff:
            self.imu_times.popleft()
            self.imu_accel_magnitudes.popleft()

    @staticmethod
    def evenly_spaced_index_sample(count, max_count):
        if count <= max_count:
            return list(range(count))
        if max_count == 1:
            return [count - 1]

        return [
            round(index * (count - 1) / (max_count - 1))
            for index in range(max_count)
        ]

    @classmethod
    def time_spaced_indices(cls, times, max_count):
        count = len(times)
        if count <= max_count:
            return list(range(count))
        if max_count == 1:
            return [count - 1]
        if any(next_time < current_time for current_time, next_time in zip(times, times[1:])):
            return cls.evenly_spaced_index_sample(count, max_count)

        start_time = times[0]
        end_time = times[-1]
        if end_time <= start_time:
            return cls.evenly_spaced_index_sample(count, max_count)

        step = (end_time - start_time) / (max_count - 1)
        indices = []
        cursor = 0
        for sample_index in range(max_count):
            target_time = start_time + step * sample_index
            while (
                cursor + 1 < count
                and abs(times[cursor + 1] - target_time) <= abs(times[cursor] - target_time)
            ):
                cursor += 1
            if not indices or cursor != indices[-1]:
                indices.append(cursor)
        return indices

    @classmethod
    def downsample_by_time(cls, times, values, max_count):
        time_list = list(times)
        if not time_list:
            return [], {
                axis: []
                for axis in values
            }

        indices = cls.time_spaced_indices(time_list, max_count)
        value_lists = {
            axis: list(axis_values)
            for axis, axis_values in values.items()
        }
        return (
            [time_list[index] for index in indices],
            {
                axis: [axis_values[index] for index in indices]
                for axis, axis_values in value_lists.items()
            },
        )

    @classmethod
    def downsample_series_by_time(cls, times, values, max_count):
        time_list = list(times)
        value_list = list(values)
        if not time_list:
            return [], []

        indices = cls.time_spaced_indices(time_list, max_count)
        return (
            [time_list[index] for index in indices],
            [value_list[index] for index in indices],
        )

    def snapshot(self):
        with self.lock:
            sampled_times = {}
            sampled_values = {}
            for line_id in self.line_ids:
                sampled_times[line_id], sampled_values[line_id] = self.downsample_by_time(
                    self.times[line_id],
                    self.values[line_id],
                    self.max_points,
                )
            sampled_imu_times, sampled_imu_accel_magnitudes = self.downsample_series_by_time(
                self.imu_times,
                self.imu_accel_magnitudes,
                self.max_points,
            )
            return (
                sampled_times,
                sampled_values,
                self.total_messages,
                dict(self.total_line_points),
                dict(self.total_plotted_points),
                sampled_imu_times,
                sampled_imu_accel_magnitudes,
                self.latest_imu_time,
                self.latest_imu_accel_magnitude,
            )


class LivoxLinePlot:
    def __init__(self, node):
        self.node = node
        self.window = pg.GraphicsLayoutWidget(show=True, title='Livox tester')
        self.window.resize(1200, 800)
        self.plots = {}
        self.curves = {}
        self.imu_plot = None
        self.imu_curve = None

        for row, axis_name in enumerate(self.node.enabled_axes):
            plot = self.window.addPlot(row=row, col=0)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel('left', axis_name)
            if axis_name == self.node.enabled_axes[-1]:
                plot.setLabel('bottom', 'Time', units='s')
            plot.setClipToView(True)
            plot.addLegend(offset=(10, 10))

            symbol = 'o' if self.node.draw_points else None
            symbol_size = 3 if self.node.draw_points else None
            self.curves[axis_name] = {}
            for line_id in self.node.line_ids:
                color = LINE_COLORS.get(line_id, pg.intColor(line_id).name())
                pen = pg.mkPen(color, width=1) if self.node.connect_points else None
                self.curves[axis_name][line_id] = plot.plot(
                    pen=pen,
                    symbol=symbol,
                    symbolSize=symbol_size,
                    symbolBrush=color if symbol else None,
                    symbolPen=None,
                    name=f'line{line_id}',
                )
            self.plots[axis_name] = plot

        if self.node.enabled_axes:
            first_axis = self.node.enabled_axes[0]
            for axis_name in self.node.enabled_axes[1:]:
                self.plots[axis_name].setXLink(self.plots[first_axis])

        self.status_label = pg.LabelItem(justify='left')
        status_row = len(self.node.enabled_axes)
        self.window.addItem(self.status_label, row=status_row, col=0)

        if self.node.show_imu:
            self.imu_plot = self.window.addPlot(row=status_row + 1, col=0)
            self.imu_plot.showGrid(x=True, y=True, alpha=0.3)
            self.imu_plot.setLabel('left', 'IMU |accel|', units='m/s^2')
            self.imu_plot.setLabel('bottom', 'Time', units='s')
            self.imu_plot.setClipToView(True)
            if self.node.enabled_axes:
                self.imu_plot.setXLink(self.plots[self.node.enabled_axes[0]])
            self.imu_curve = self.imu_plot.plot(
                pen=pg.mkPen('#ffffff', width=1.5),
                name='IMU |accel|',
            )

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.node.update_interval_ms)

    def update(self):
        (
            times,
            values,
            total_messages,
            total_line_points,
            total_plotted_points,
            imu_times,
            imu_accel_magnitudes,
            latest_imu_time,
            latest_imu_accel_magnitude,
        ) = self.node.snapshot()

        latest_times = [
            line_times[-1]
            for line_times in times.values()
            if line_times
        ]
        if imu_times:
            latest_times.append(imu_times[-1])

        if not latest_times:
            return

        latest_time = max(latest_times)
        x_min = max(0.0, latest_time - self.node.window_s)
        x_max = x_min + self.node.window_s

        for axis_name, line_curves in self.curves.items():
            for line_id, curve in line_curves.items():
                curve.setData(times[line_id], values[line_id][axis_name])
            self.plots[axis_name].setXRange(x_min, x_max, padding=0)

        if self.node.show_imu:
            self.imu_curve.setData(imu_times, imu_accel_magnitudes)
            self.imu_plot.setXRange(x_min, x_max, padding=0)

        imu_text = 'IMU waiting'
        if latest_imu_time is not None and latest_imu_accel_magnitude is not None:
            imu_text = f'IMU |accel| {latest_imu_accel_magnitude:.3f} m/s^2'

        point_text = ', '.join(
            f'line{line_id}: {total_plotted_points[line_id]}/{total_line_points[line_id]}'
            for line_id in self.node.line_ids
        )
        displayed_points = sum(len(line_times) for line_times in times.values())
        self.status_label.setText(
            f'Livox lines {self.node.line_ids} xyz | clouds {total_messages} | '
            f'plotted/input {point_text} | displayed {displayed_points} | {imu_text}'
            f' | imu samples {len(imu_times)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxTester()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    app = pg.mkQApp('livox_tester')
    plot = LivoxLinePlot(node)

    def handle_sigint(_signum, _frame):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        QtWidgets.QApplication.instance().exec_()
    finally:
        plot.timer.stop()
        plot.window.close()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
