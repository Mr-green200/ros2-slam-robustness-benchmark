import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.005)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('laps', 3)
        self.declare_parameter('waypoints', [
            2.6, 0.0,
            2.6, 2.1,
            1.4, 2.0,
            1.4, 0.0,
        ])

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.start_delay = self.get_parameter('start_delay').value
        self.max_laps = int(self.get_parameter('laps').value)

        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = []
        for i in range(0, len(wp_flat), 2):
            self.waypoints.append((wp_flat[i], wp_flat[i+1]))

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/benchmark/navigation_done', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.current_wp_idx = 0
        self.lap_count = 0
        self.state = 'WAITING'
        self.start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Navigator: {len(self.waypoints)} waypoints, '
            f'{self.max_laps} laps, speed={self.linear_speed}m/s'
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.odom_received = True

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def get_target_angle(self, tx, ty):
        return math.atan2(ty - self.current_y, tx - self.current_x)

    def get_distance(self, tx, ty):
        dx = tx - self.current_x
        dy = ty - self.current_y
        return math.sqrt(dx*dx + dy*dy)

    def normalize_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def control_loop(self):
        if not self.odom_received:
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.state == 'WAITING':
            if elapsed < self.start_delay:
                return
            self.get_logger().info('Starting navigation!')
            self.state = 'ROTATING'
            return

        if self.state == 'DONE':
            self.publish_cmd()
            return

        if self.current_wp_idx >= len(self.waypoints):
            self.lap_count += 1
            self.get_logger().info(f'Lap {self.lap_count}/{self.max_laps} complete')
            if self.lap_count >= self.max_laps:
                self.state = 'DONE'
                self.get_logger().info('All laps complete!')
                self.publish_cmd()
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
                self.create_timer(1.0, self.publish_done)
                return
            self.current_wp_idx = 0

        tx, ty = self.waypoints[self.current_wp_idx]
        target_angle = self.get_target_angle(tx, ty)
        distance = self.get_distance(tx, ty)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        if self.state == 'ROTATING':
            if abs(angle_error) < self.angle_tolerance:
                self.state = 'DRIVING'
                return
            speed = max(-self.angular_speed, min(self.angular_speed, angle_error * 4.0))
            self.publish_cmd(angular_z=speed)

        elif self.state == 'DRIVING':
            if distance < self.wp_tolerance:
                self.get_logger().info(
                    f'WP {self.current_wp_idx}: ({tx:.1f}, {ty:.1f}) reached'
                )
                self.current_wp_idx += 1
                self.state = 'ROTATING'
                self.publish_cmd()
                return

            if abs(angle_error) > 0.05:
                self.state = 'ROTATING'
                return

            self.publish_cmd(linear_x=self.linear_speed, angular_z=angle_error * 3.0)

    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
