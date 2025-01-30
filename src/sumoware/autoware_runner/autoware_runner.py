import threading
import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String

class AutowareRunner(Node):

    def __init__(self):
        super().__init__('autoware_runner')

        self.declare_parameter('project_absolute_path', '')
        self.project_absolute_path = self.get_parameter('project_absolute_path').value

        self.declare_parameter('gpu_support', False)
        self.gpu_support = self.get_parameter('gpu_support').value

        if not self.project_absolute_path:
            raise ValueError("Project absolute path not provided")

        self.process = self.run_autoware_process()
        self.create_subscription(
            String,
            '/sumoware/shutdown',
            self.shutdown_callback,
            10
        )
    
    def shutdown_callback(self, msg):
        self.get_logger().info('Shutting down...')
        subprocess.run(['docker', 'kill', 'autoware'])
        raise SystemExit

    def run_autoware_process(self):
        cmd = [
            'rocker', '--name', 'autoware', '--nvidia', '--x11',
            '--user', '--volume', f'{self.project_absolute_path}/map:/map', '--volume',
            f'{self.project_absolute_path}/launcher_autoware:/launcher_autoware', '--',
            'ghcr.io/autowarefoundation/autoware:universe-cuda-20241010-amd64',
            'ros2', 'launch', '/launcher_autoware/planning_simulator.launch.xml',
            'map_path:=/map', 'vehicle_model:=sample_vehicle', 'sensor_model:=sample_sensor_kit',
            'use_sim_time:=true'
        ]
        if not self.gpu_support:
            cmd.remove('--nvidia')
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        threading.Thread(target=self.read_stdout, args=(process.stdout,), daemon=True).start()
        threading.Thread(target=self.read_stderr, args=(process.stderr,), daemon=True).start()

        return process

    def read_stdout(self, pipe):
        for line in iter(pipe.readline, ''):
            self.get_logger().info(line.strip())
        pipe.close()

    def read_stderr(self, pipe):
        for line in iter(pipe.readline, ''):
            self.get_logger().error(line.strip())
        pipe.close()


def main(args=None):
    rclpy.init(args=args)

    autoware_runner = AutowareRunner()

    try:
        rclpy.spin(autoware_runner)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        autoware_runner.destroy_node()
        rclpy.shutdown()

