#!/usr/bin/env python3

import math
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
from sensor_msgs.msg import Imu

try:
    from unitree_go.msg import LowState
except ImportError as exc:
    LowState = None
    LOWSTATE_IMPORT_ERROR = exc
else:
    LOWSTATE_IMPORT_ERROR = None


FOOT_LABELS = ['FL', 'FR', 'RL', 'RR']


class RealtimeFootForce(Node):
    def __init__(self):
        super().__init__('realtime_foot_force')

        if LowState is None:
            raise RuntimeError(
                'Cannot import unitree_go.msg.LowState. Source the workspace or install unitree_go.'
            ) from LOWSTATE_IMPORT_ERROR

        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('livox_imu_topic', '/livox/imu')
        self.declare_parameter('livox_accel_scale', 9.80665)
        self.declare_parameter('go2_accel_scale', 1.04)  # Scale GO2 IMU accel to match Livox at rest
        self.declare_parameter('window_s', 1.0)
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('update_interval_ms', 10)

        self.lowstate_topic = self.get_parameter('lowstate_topic').value
        self.livox_imu_topic = self.get_parameter('livox_imu_topic').value
        self.livox_accel_scale = float(self.get_parameter('livox_accel_scale').value)
        self.go2_accel_scale = float(self.get_parameter('go2_accel_scale').value)
        self.window_s = float(self.get_parameter('window_s').value)
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.update_interval_ms = int(self.get_parameter('update_interval_ms').value)

        if self.window_s <= 0.0:
            raise ValueError('window_s must be greater than 0.0')
        if self.time_scale <= 0.0:
            raise ValueError('time_scale must be greater than 0.0')
        if self.update_interval_ms <= 0:
            raise ValueError('update_interval_ms must be greater than 0')

        self.lock = threading.Lock()
        self.start_time = None
        self.times = deque()
        self.foot_forces = [deque() for _ in FOOT_LABELS]
        self.go2_imu_times = deque()
        self.go2_accel_magnitudes = deque()
        self.livox_imu_times = deque()
        self.livox_accel_magnitudes = deque()

        self.lowstate_subscription = self.create_subscription(
            LowState,
            self.lowstate_topic,
            self.lowstate_callback,
            qos_profile_sensor_data,
        )
        self.livox_imu_subscription = self.create_subscription(
            Imu,
            self.livox_imu_topic,
            self.livox_imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Listening to {self.lowstate_topic} and {self.livox_imu_topic}; '
            f'plotting a {self.window_s:.1f}s rolling window at {self.time_scale:.2f}x time scale'
        )

    def lowstate_callback(self, msg):
        foot_force = self.extract_foot_force(msg)
        if foot_force is None:
            self.get_logger().warn(
                'LowState message does not contain four foot_force values',
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = self.elapsed_time(now)
        accel_magnitude = self.extract_go2_accel_magnitude(msg)

        with self.lock:
            self.times.append(elapsed)
            for values, value in zip(self.foot_forces, foot_force):
                values.append(float(value))
            self.trim_force_locked(elapsed - self.window_s)

            if accel_magnitude is not None:
                self.go2_imu_times.append(elapsed)
                self.go2_accel_magnitudes.append(accel_magnitude * self.go2_accel_scale)
                self.trim_imu_locked(
                    self.go2_imu_times,
                    self.go2_accel_magnitudes,
                    elapsed - self.window_s,
                )

    def livox_imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = self.elapsed_time(now)
        accel_magnitude = self.extract_livox_accel_magnitude(msg)

        with self.lock:
            self.livox_imu_times.append(elapsed)
            self.livox_accel_magnitudes.append(accel_magnitude * self.livox_accel_scale)
            self.trim_imu_locked(
                self.livox_imu_times,
                self.livox_accel_magnitudes,
                elapsed - self.window_s,
            )

    def elapsed_time(self, now):
        with self.lock:
            if self.start_time is None:
                self.start_time = now
            return (now - self.start_time) * self.time_scale

    def trim_force_locked(self, cutoff):
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            for values in self.foot_forces:
                values.popleft()

    @staticmethod
    def trim_imu_locked(times, values, cutoff):
        while times and times[0] < cutoff:
            times.popleft()
            values.popleft()

    @staticmethod
    def extract_foot_force(msg):
        if not hasattr(msg, 'foot_force'):
            return None

        values = list(msg.foot_force)
        if len(values) < 4:
            return None
        return values[:4]

    @staticmethod
    def extract_go2_accel_magnitude(msg):
        accel = None
        if hasattr(msg, 'imu_state') and hasattr(msg.imu_state, 'accelerometer'):
            accel = msg.imu_state.accelerometer
        elif hasattr(msg, 'accelerometer'):
            accel = msg.accelerometer

        if accel is None:
            return None

        values = list(accel)
        if len(values) < 3:
            return None

        return math.sqrt(sum(float(value) * float(value) for value in values[:3]))

    @staticmethod
    def extract_livox_accel_magnitude(msg):
        accel = msg.linear_acceleration
        return math.sqrt(
            float(accel.x) * float(accel.x)
            + float(accel.y) * float(accel.y)
            + float(accel.z) * float(accel.z)
        )

    def snapshot(self):
        with self.lock:
            return (
                list(self.times),
                [list(values) for values in self.foot_forces],
                list(self.go2_imu_times),
                list(self.go2_accel_magnitudes),
                list(self.livox_imu_times),
                list(self.livox_accel_magnitudes),
            )


class FootForcePlot:
    def __init__(self, node):
        self.node = node

        self.figure, (self.force_axis, self.imu_axis) = plt.subplots(
            2,
            1,
            figsize=(12, 8),
            sharex=True,
            gridspec_kw={'height_ratios': [2, 1]},
        )
        self.force_lines = [
            self.force_axis.plot([], [], label=label, linewidth=1.2)[0]
            for label in FOOT_LABELS
        ]
        self.go2_accel_line = self.imu_axis.plot(
            [],
            [],
            label='GO2 |accel|',
            color='black',
            linestyle='--',
            linewidth=1.4,
        )[0]
        self.livox_accel_line = self.imu_axis.plot(
            [],
            [],
            label='Livox |accel|',
            color='tab:purple',
            linewidth=1.2,
        )[0]

        self.force_axis.set_ylabel('Foot force')
        self.force_axis.set_title('GO2 Foot Force')
        self.force_axis.grid(True, alpha=0.3)
        self.force_axis.legend(loc='upper right')

        self.imu_axis.set_xlabel('Scaled time (s)')
        self.imu_axis.set_ylabel('|accel|')
        self.imu_axis.set_title('IMU Accelerometer Magnitude')
        self.imu_axis.grid(True, alpha=0.3)
        self.imu_axis.xaxis.set_major_locator(MultipleLocator(0.1))
        self.imu_axis.xaxis.set_minor_locator(MultipleLocator(0.01))
        self.imu_axis.grid(True, which='minor', axis='x', alpha=0.12)
        self.imu_axis.legend(loc='upper right')
        self.figure.tight_layout()

    def update(self, _frame):
        (
            times,
            foot_forces,
            go2_imu_times,
            go2_accel_magnitudes,
            livox_imu_times,
            livox_accel_magnitudes,
        ) = self.node.snapshot()
        latest_times = [values[-1] for values in (times, go2_imu_times, livox_imu_times) if values]
        if not latest_times:
            return [*self.force_lines, self.go2_accel_line, self.livox_accel_line]

        latest_time = max(latest_times)
        x_min = max(0.0, latest_time - self.node.window_s)
        x_max = x_min + self.node.window_s

        for line, values in zip(self.force_lines, foot_forces):
            line.set_data(times, values)

        self.go2_accel_line.set_data(go2_imu_times, go2_accel_magnitudes)
        scaled_livox_accel = [
            value
            for value in livox_accel_magnitudes
        ]
        self.livox_accel_line.set_data(livox_imu_times, scaled_livox_accel)

        self.force_axis.set_xlim(x_min, x_max)
        self.force_axis.relim()
        self.force_axis.autoscale_view(scalex=False, scaley=True)

        self.imu_axis.set_xlim(x_min, x_max)
        self.imu_axis.relim()
        self.imu_axis.autoscale_view(scalex=False, scaley=True)
        return [*self.force_lines, self.go2_accel_line, self.livox_accel_line]


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeFootForce()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    plot = FootForcePlot(node)
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
