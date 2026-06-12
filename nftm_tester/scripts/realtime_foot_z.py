#!/usr/bin/env python3

import os
import signal
import threading
from collections import deque

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MultipleLocator

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray


DEFAULT_FOOT_LABELS = ['fr', 'fl', 'rr', 'rl']


class RealtimeFootZ(Node):
    def __init__(self):
        super().__init__('realtime_foot_z')

        self.declare_parameter('foot_marker_topic', '/fastlio2/foot_markers')
        self.declare_parameter('foot_labels', DEFAULT_FOOT_LABELS)
        self.declare_parameter('window_s', 5.0)
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('update_interval_ms', 30)
        self.declare_parameter('y_min', -30.0)
        self.declare_parameter('y_max', -10.0)
        self.declare_parameter('z_scale', 100.0)
        self.declare_parameter('z_unit', 'cm')

        self.foot_marker_topic = self.get_parameter('foot_marker_topic').value
        self.foot_labels = list(self.get_parameter('foot_labels').value)
        self.window_s = float(self.get_parameter('window_s').value)
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.update_interval_ms = int(self.get_parameter('update_interval_ms').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.z_scale = float(self.get_parameter('z_scale').value)
        self.z_unit = str(self.get_parameter('z_unit').value)

        if len(self.foot_labels) != 4:
            raise ValueError('foot_labels must contain exactly four labels')
        if self.window_s <= 0.0:
            raise ValueError('window_s must be greater than 0.0')
        if self.time_scale <= 0.0:
            raise ValueError('time_scale must be greater than 0.0')
        if self.update_interval_ms <= 0:
            raise ValueError('update_interval_ms must be greater than 0')
        if self.y_min >= self.y_max:
            raise ValueError('y_min must be less than y_max')
        if self.z_scale == 0.0:
            raise ValueError('z_scale must not be 0.0')

        self.lock = threading.Lock()
        self.start_time = None
        self.times = deque()
        self.foot_z_values = [deque() for _ in self.foot_labels]
        self.history_high = None
        self.history_low = None

        self.subscription = self.create_subscription(
            MarkerArray,
            self.foot_marker_topic,
            self.marker_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Listening to {self.foot_marker_topic}; plotting foot z positions '
            f'for {", ".join(self.foot_labels)} over a {self.window_s:.1f}s rolling window'
        )

    def marker_callback(self, msg):
        z_values = self.extract_z_values(msg)
        if z_values is None:
            self.get_logger().warn(
                'MarkerArray does not contain four foot markers with usable positions',
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = self.elapsed_time(now)

        with self.lock:
            self.times.append(elapsed)
            for values, z_value in zip(self.foot_z_values, z_values):
                z_value = float(z_value) * self.z_scale
                values.append(z_value)
                if self.history_high is None or z_value > self.history_high:
                    self.history_high = z_value
                if self.history_low is None or z_value < self.history_low:
                    self.history_low = z_value
            self.trim_locked(elapsed - self.window_s)

    def elapsed_time(self, now):
        with self.lock:
            if self.start_time is None:
                self.start_time = now
            return (now - self.start_time) * self.time_scale

    def trim_locked(self, cutoff):
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            for values in self.foot_z_values:
                values.popleft()

    @staticmethod
    def extract_z_values(msg):
        markers = [marker for marker in msg.markers if marker.action != marker.DELETE]
        if len(markers) < 4:
            return None

        markers = sorted(markers, key=lambda marker: marker.id)[:4]
        return [marker.pose.position.z for marker in markers]

    def snapshot(self):
        with self.lock:
            return (
                list(self.times),
                [list(values) for values in self.foot_z_values],
                self.history_high,
                self.history_low,
            )


class FootZPlot:
    def __init__(self, node):
        self.node = node

        self.figure, self.axis = plt.subplots(figsize=(12, 6))
        self.lines = [
            self.axis.plot([], [], label=label, linewidth=1.4)[0]
            for label in self.node.foot_labels
        ]
        self.history_high_line = self.axis.axhline(
            0.0,
            label='history max',
            color='black',
            linestyle='--',
            linewidth=1.2,
            visible=False,
        )
        self.history_low_line = self.axis.axhline(
            0.0,
            label='history min',
            color='black',
            linestyle=':',
            linewidth=1.2,
            visible=False,
        )

        self.axis.set_xlabel('Time (s)')
        self.axis.set_ylabel(f'World z position ({self.node.z_unit})')
        self.axis.set_title('FASTLIO2 Foot Z Positions')
        self.axis.set_ylim(self.node.y_min, self.node.y_max)
        self.axis.grid(True, alpha=0.3)
        self.axis.xaxis.set_major_locator(MultipleLocator(0.5))
        self.axis.xaxis.set_minor_locator(MultipleLocator(0.1))
        self.axis.grid(True, which='minor', axis='x', alpha=0.12)
        self.axis.legend(loc='upper right')
        self.figure.tight_layout()

    def update(self, _frame):
        times, foot_z_values, history_high, history_low = self.node.snapshot()
        if not times:
            return [*self.lines, self.history_high_line, self.history_low_line]

        latest_time = times[-1]
        x_min = max(0.0, latest_time - self.node.window_s)
        x_max = x_min + self.node.window_s

        for line, values in zip(self.lines, foot_z_values):
            line.set_data(times, values)

        if history_high is not None:
            self.history_high_line.set_ydata([history_high, history_high])
            self.history_high_line.set_visible(True)
        if history_low is not None:
            self.history_low_line.set_ydata([history_low, history_low])
            self.history_low_line.set_visible(True)

        self.axis.set_xlim(x_min, x_max)
        self.axis.set_ylim(self.node.y_min, self.node.y_max)
        return [*self.lines, self.history_high_line, self.history_low_line]


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeFootZ()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    plot = FootZPlot(node)
    animation = FuncAnimation(
        plot.figure,
        plot.update,
        interval=node.update_interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    def handle_sigint(_signum, _frame):
        plt.close(plot.figure)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        plt.show()
    finally:
        animation.event_source.stop()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
