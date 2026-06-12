#!/usr/bin/env python3

import math
from typing import cast

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
import tf2_py as tf2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


TF2_TRANSFORM_EXCEPTION = cast(type[Exception], getattr(tf2, 'TransformException', Exception))


def quat_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    q = quat_normalize(q)
    q_vec = (v[0], v[1], v[2], 0.0)
    q_conj = (-q[0], -q[1], -q[2], q[3])
    rotated = quat_multiply(quat_multiply(q, q_vec), q_conj)
    return rotated[:3]


def transform_to_parts(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        (translation.x, translation.y, translation.z),
        quat_normalize((rotation.x, rotation.y, rotation.z, rotation.w)),
    )


def compose(a, b):
    a_t, a_q = a
    b_t, b_q = b
    b_t_in_a = quat_rotate(a_q, b_t)
    return (
        (
            a_t[0] + b_t_in_a[0],
            a_t[1] + b_t_in_a[1],
            a_t[2] + b_t_in_a[2],
        ),
        quat_normalize(quat_multiply(a_q, b_q)),
    )


class FastlioLidarToBaseTF(Node):
    def __init__(self):
        super().__init__('fastlio_lidar_to_base_tf')

        self.world_frame: str = self._declare_str_param('world_frame', 'world')
        self.fastlio_lidar_frame: str = self._declare_str_param(
            'fastlio_lidar_frame', 'fastlio_lidar'
        )
        self.lidar_imu_frame: str = self._declare_str_param(
            'lidar_imu_frame', 'mid360_imu'
        )
        self.base_frame: str = self._declare_str_param('base_frame', 'base')
        publish_rate = self._declare_float_param('publish_rate', 10.0)
        self.shutting_down = False
        self.timer = None
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_broadcaster = None
        lookup_timeout_sec = self._declare_float_param('lookup_timeout_sec', 0.05)
        self.lookup_timeout = Duration(
            nanoseconds=int(lookup_timeout_sec * 1_000_000_000)
        )
        self.use_fastlio_stamp: bool = self._declare_bool_param(
            'use_fastlio_stamp', False
        )

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than 0.0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_world_to_base)

        self.get_logger().info(
            f'Publishing {self.world_frame} -> {self.base_frame} from '
            f'{self.world_frame} -> {self.fastlio_lidar_frame} and '
            f'{self.lidar_imu_frame} -> {self.base_frame}'
        )

    def _declare_str_param(self, name: str, default_value: str) -> str:
        value = self.declare_parameter(name, default_value).value
        if not isinstance(value, str):
            raise TypeError(f'Parameter {name} must be a string')
        return value

    def _declare_float_param(self, name: str, default_value: float) -> float:
        value = self.declare_parameter(name, default_value).value
        if not isinstance(value, (int, float)):
            raise TypeError(f'Parameter {name} must be numeric')
        return float(cast(int | float, value))

    def _declare_bool_param(self, name: str, default_value: bool) -> bool:
        value = self.declare_parameter(name, default_value).value
        if not isinstance(value, bool):
            raise TypeError(f'Parameter {name} must be a bool')
        return value

    def publish_world_to_base(self):
        if (
            self.shutting_down
            or self.tf_buffer is None
            or self.tf_broadcaster is None
            or not self.context.ok()
        ):
            return

        try:
            world_to_fastlio_lidar = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.fastlio_lidar_frame,
                Time(),
                self.lookup_timeout,
            )
            robot_lidar_to_base = self.tf_buffer.lookup_transform(
                self.lidar_imu_frame,
                self.base_frame,
                Time(),
                self.lookup_timeout,
            )
        except TF2_TRANSFORM_EXCEPTION as exc:
            self.get_logger().warn(
                f'Waiting for TF chain: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        translation, rotation = compose(
            transform_to_parts(world_to_fastlio_lidar),
            transform_to_parts(robot_lidar_to_base),
        )

        transform = TransformStamped()
        if self.use_fastlio_stamp:
            transform.header.stamp = world_to_fastlio_lidar.header.stamp
        else:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(transform)

    def cleanup(self):
        self.shutting_down = True

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None

        self.tf_broadcaster = None
        self.tf_buffer = None


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = FastlioLidarToBaseTF()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.context.ok():
                node.get_logger().info('Shutting down fastlio lidar to base TF publisher')
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
