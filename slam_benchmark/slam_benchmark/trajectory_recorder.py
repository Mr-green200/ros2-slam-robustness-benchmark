import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros
import os
import time
import subprocess
import signal


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('output_dir', '/tmp/slam_benchmark_results')
        self.declare_parameter('run_id', 'run_001')
        self.declare_parameter('algorithm', 'slam_toolbox')
        self.declare_parameter('noise_pct', 0)
        self.declare_parameter('world', 'loop_closure')
        self.declare_parameter('duration_sec', 120)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.run_id = str(self.get_parameter('run_id').value)
        self.algorithm = str(self.get_parameter('algorithm').value)
        self.noise_pct = int(self.get_parameter('noise_pct').value)
        self.world = str(self.get_parameter('world').value)
        self.duration = float(self.get_parameter('duration_sec').value)

        self.result_dir = os.path.join(
            self.output_dir, self.world, self.algorithm,
            f'noise_{self.noise_pct}pct', self.run_id
        )
        os.makedirs(self.result_dir, exist_ok=True)

        self.gt_file = open(os.path.join(self.result_dir, 'ground_truth.txt'), 'w')
        self.est_file = open(os.path.join(self.result_dir, 'estimated.txt'), 'w')
        self.odom_file = open(os.path.join(self.result_dir, 'odometry.txt'), 'w')

        for f in [self.gt_file, self.est_file, self.odom_file]:
            f.write('# TUM format: timestamp x y z qx qy qz qw\n')

        self.create_subscription(PoseStamped, '/ground_truth_pose', self.gt_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/benchmark/navigation_done', self.nav_done_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.sample_estimated_pose)

        self.gt_count = 0
        self.est_count = 0
        self.start_time = time.time()
        self.finished = False
        self.stop_reason = ''

        self.create_timer(5.0, self.status_update)

        self.get_logger().info(
            f'Recorder: {self.algorithm}, noise={self.noise_pct}%, '
            f'duration={self.duration}s (max), stops when laps complete or duration reached'
        )

    def write_tum_line(self, f, timestamp, x, y, z, qx, qy, qz, qw):
        f.write(f'{timestamp:.9f} {x:.6f} {y:.6f} {z:.6f} '
                f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
        f.flush()

    def gt_callback(self, msg: PoseStamped):
        if self.finished:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        o = msg.pose.orientation
        self.write_tum_line(self.gt_file, t, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        self.gt_count += 1

    def odom_callback(self, msg: Odometry):
        if self.finished:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.write_tum_line(self.odom_file, t, p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def sample_estimated_pose(self):
        if self.finished:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            t_stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            tr = transform.transform.translation
            ro = transform.transform.rotation
            self.write_tum_line(self.est_file, t_stamp, tr.x, tr.y, tr.z,
                                ro.x, ro.y, ro.z, ro.w)
            self.est_count += 1
        except Exception:
            pass

    def destroy_node(self):
        if not self.finished:
            self._kill_all()
        super().destroy_node()

    def _kill_all(self):
        targets = [
            'gz sim', 'gzserver', 'gz-transport-topic',
            'async_slam_toolbox_node', 'nav2_amcl', 'nav2_map_server',
            'waypoint_navigator', 'gt_publisher', 'tf_noise_injector',
            'robot_state_publisher',
            'parameter_bridge', 'image_bridge',
        ]
        for target in targets:
            subprocess.run(['pkill', '-9', '-f', target], capture_output=True)
        subprocess.run(['pkill', '-9', 'gz'], capture_output=True)

    def nav_done_callback(self, msg: Bool):
        if msg.data and not self.finished:
            self.stop_reason = 'ALL LAPS COMPLETE'
            self.finish_and_evaluate()

    def status_update(self):
        if self.finished:
            return
        elapsed = time.time() - self.start_time
        remaining = max(0, self.duration - elapsed)
        self.get_logger().info(
            f'[{elapsed:.0f}s/{self.duration:.0f}s] GT={self.gt_count} '
            f'EST={self.est_count} | {remaining:.0f}s remaining'
        )
        if elapsed >= self.duration:
            self.stop_reason = 'DURATION TIMEOUT'
            self.finish_and_evaluate()

    def finish_and_evaluate(self):
        """Write metadata, run evo_ape/evo_rpe, print summary, then kill all processes."""
        self.finished = True
        elapsed = time.time() - self.start_time
        self.gt_file.close()
        self.est_file.close()
        self.odom_file.close()

        meta_path = os.path.join(self.result_dir, 'metadata.txt')
        with open(meta_path, 'w') as f:
            f.write(f'algorithm: {self.algorithm}\n')
            f.write(f'noise_pct: {self.noise_pct}\n')
            f.write(f'world: {self.world}\n')
            f.write(f'run_id: {self.run_id}\n')
            f.write(f'duration: {elapsed:.1f}\n')
            f.write(f'max_duration: {self.duration}\n')
            f.write(f'stop_reason: {self.stop_reason}\n')
            f.write(f'gt_samples: {self.gt_count}\n')
            f.write(f'est_samples: {self.est_count}\n')

        gt_path = os.path.join(self.result_dir, 'ground_truth.txt')
        est_path = os.path.join(self.result_dir, 'estimated.txt')

        ate_text = ''
        rpe_text = ''

        try:
            r = subprocess.run(
                ['evo_ape', 'tum', gt_path, est_path,
                 '--align', '--no_warnings'],
                capture_output=True, text=True, timeout=30
            )
            ate_text = r.stdout.strip()
            with open(os.path.join(self.result_dir, 'ate_results.txt'), 'w') as f:
                f.write(ate_text)
        except Exception:
            ate_text = 'ATE evaluation failed'

        try:
            r = subprocess.run(
                ['evo_rpe', 'tum', gt_path, est_path,
                 '--align', '--no_warnings',
                 '--delta', '1', '--delta_unit', 'm'],
                capture_output=True, text=True, timeout=30
            )
            rpe_text = r.stdout.strip()
            with open(os.path.join(self.result_dir, 'rpe_results.txt'), 'w') as f:
                f.write(rpe_text)
        except Exception:
            rpe_text = 'RPE evaluation failed'

        try:
            subprocess.run(
                ['evo_ape', 'tum', gt_path, est_path,
                 '--align', '--no_warnings',
                 '--save_plot', os.path.join(self.result_dir, 'ate_plot.pdf'),
                 '--plot_mode', 'xy'],
                capture_output=True, text=True, timeout=30
            )
        except Exception:
            pass

        summary = f"""
╔══════════════════════════════════════════════════════════════╗
║                   BENCHMARK COMPLETE                        ║
╠══════════════════════════════════════════════════════════════╣
║  Algorithm:    {self.algorithm:<44}║
║  Noise:        {self.noise_pct}%{' '*(43-len(str(self.noise_pct)))}║
║  World:        {self.world:<44}║
║  Run ID:       {self.run_id:<44}║
║  Duration:     {elapsed:.1f}s (max {self.duration:.0f}s){' '*(30-len(f'{elapsed:.1f}s (max {self.duration:.0f}s)'))}║
║  Stop Reason:  {self.stop_reason:<44}║
║  GT Samples:   {self.gt_count:<44}║
║  EST Samples:  {self.est_count:<44}║
╠══════════════════════════════════════════════════════════════╣
║  ATE (Absolute Trajectory Error):                           ║
╠══════════════════════════════════════════════════════════════╣
{self.indent_text(ate_text)}
╠══════════════════════════════════════════════════════════════╣
║  RPE (Relative Pose Error):                                 ║
╠══════════════════════════════════════════════════════════════╣
{self.indent_text(rpe_text)}
╠══════════════════════════════════════════════════════════════╣
║  Results: {self.result_dir:<49}║
╚══════════════════════════════════════════════════════════════╝
"""
        print(summary, flush=True)
        time.sleep(2)
        self._kill_all()
        time.sleep(1)
        os.kill(os.getppid(), signal.SIGINT)

    def indent_text(self, text):
        lines = text.split('\n')
        result = ''
        for line in lines:
            if line.strip():
                padded = f'║  {line:<58}║'
                result += padded + '\n'
        return result.rstrip('\n')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
