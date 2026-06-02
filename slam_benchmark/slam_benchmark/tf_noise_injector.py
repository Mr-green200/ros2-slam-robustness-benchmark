import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfNoiseInjector(Node):
    def __init__(self):
        super().__init__('tf_noise_injector')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('noise_percentage', 0)
        self.declare_parameter('seed', 200)

        self.noise_pct = float(self.get_parameter('noise_percentage').value) / 100.0
        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(seed)

        self.get_logger().info(
            f'TF Noise Injector started: noise={self.noise_pct * 100:.1f}%, seed={seed}. '
            f'Subscribing to /tf_bridge, publishing to /tf.'
        )

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(TFMessage, '/tf_bridge', self.tf_callback, qos)
        self.pub = self.create_publisher(TFMessage, '/tf', qos)

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.acc_noise_x = 0.0
        self.acc_noise_y = 0.0
        self.acc_noise_yaw = 0.0
        self.first_msg = True
        self.odom_count = 0

    def _yaw_from_quat(self, q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _apply_noise(self, transform):
        """Accumulate distance-proportional drift onto an odom→base_footprint transform."""
        tr = transform.transform.translation
        curr_x, curr_y = tr.x, tr.y
        curr_yaw = self._yaw_from_quat(transform.transform.rotation)

        if self.first_msg:
            self.prev_x, self.prev_y, self.prev_yaw = curr_x, curr_y, curr_yaw
            self.first_msg = False
            return

        dx = curr_x - self.prev_x
        dy = curr_y - self.prev_y
        dyaw = curr_yaw - self.prev_yaw
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi

        dist = math.sqrt(dx * dx + dy * dy)

        if dist > 1e-6:
            self.acc_noise_x += self.rng.normal(0, dist * self.noise_pct)
            self.acc_noise_y += self.rng.normal(0, dist * self.noise_pct * 0.3)
        if abs(dyaw) > 1e-6:
            self.acc_noise_yaw += self.rng.normal(0, abs(dyaw) * self.noise_pct)

        self.prev_x, self.prev_y, self.prev_yaw = curr_x, curr_y, curr_yaw

        tr.x = curr_x + self.acc_noise_x
        tr.y = curr_y + self.acc_noise_y
        noisy_yaw = curr_yaw + self.acc_noise_yaw
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(noisy_yaw / 2.0)
        transform.transform.rotation.w = math.cos(noisy_yaw / 2.0)

        self.odom_count += 1
        if self.odom_count % 250 == 0:
            self.get_logger().info(
                f'Accumulated noise: x={self.acc_noise_x:.4f}m, '
                f'y={self.acc_noise_y:.4f}m, '
                f'yaw={math.degrees(self.acc_noise_yaw):.2f}deg'
            )

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if (transform.header.frame_id == 'odom'
                    and transform.child_frame_id == 'base_footprint'
                    and self.noise_pct > 0.0):
                self._apply_noise(transform)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfNoiseInjector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
