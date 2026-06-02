import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import json
import threading


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.declare_parameter('model_name', 'waffle')
        self.declare_parameter('world_name', 'default')

        self.model_name = self.get_parameter('model_name').value
        self.world_name = self.get_parameter('world_name').value

        self.publisher = self.create_publisher(PoseStamped, '/ground_truth_pose', 10)
        self.get_logger().info(
            f'Ground Truth Publisher: model={self.model_name}, world={self.world_name}'
        )

        self.gz_thread = threading.Thread(target=self.read_gz_poses, daemon=True)
        self.gz_thread.start()

    def destroy_node(self):
        if hasattr(self, '_gz_proc') and self._gz_proc.poll() is None:
            self._gz_proc.kill()
        super().destroy_node()

    def read_gz_poses(self):
        """Stream Gazebo pose/info topic and forward matching model poses."""
        topic = f'/world/{self.world_name}/pose/info'
        self._gz_proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', topic, '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
        )
        buffer = ''
        depth = 0
        for char in iter(lambda: self._gz_proc.stdout.read(1), ''):
            buffer += char
            if char == '{':
                depth += 1
            elif char == '}':
                depth -= 1
                if depth == 0:
                    try:
                        data = json.loads(buffer)
                        self.process_pose_data(data)
                    except json.JSONDecodeError:
                        pass
                    buffer = ''

    def process_pose_data(self, data):
        """Publish the pose of the tracked model as PoseStamped on /ground_truth_pose."""
        header = data.get('header', {})
        stamp = header.get('stamp', {})
        sec = int(stamp.get('sec', 0))
        nsec = int(stamp.get('nsec', 0))

        for pose in data.get('pose', []):
            if pose.get('name') == self.model_name:
                msg = PoseStamped()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nsec
                msg.header.frame_id = 'world'

                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                msg.pose.position.x = pos.get('x', 0.0)
                msg.pose.position.y = pos.get('y', 0.0)
                msg.pose.position.z = pos.get('z', 0.0)
                msg.pose.orientation.x = ori.get('x', 0.0)
                msg.pose.orientation.y = ori.get('y', 0.0)
                msg.pose.orientation.z = ori.get('z', 0.0)
                msg.pose.orientation.w = ori.get('w', 1.0)

                self.publisher.publish(msg)
                break


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
