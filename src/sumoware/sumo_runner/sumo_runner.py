import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String

class SumoRunner(Node):

    def __init__(self):
        super().__init__('sumo_runner')

        self.declare_parameter('port', 4001)
        self.declare_parameter('step_length', 0.01)
        self.declare_parameter('project_absolute_path', '')

        port = self.get_parameter('port').value
        step_length = self.get_parameter('step_length').value

        self.project_absolute_path = self.get_parameter('project_absolute_path').value

        if not self.project_absolute_path:
            raise ValueError("Project absolute path not provided")

        self.process = self.run_sumo_process(port, step_length)

        self.create_subscription(
            String,
            '/sumoware/shutdown',
            self.shutdown_callback,
            10
        )
    
    def shutdown_callback(self, msg):
        self.get_logger().info('Shutting down...')
        subprocess.run(['docker', 'kill', 'sumo-gui'])
        raise SystemExit

    def run_sumo_process(self, port, step_length):
        cmd = [
            'docker', 'run', '--name', 'sumo-gui', '-e', 'DISPLAY=:0',
            '-v', '/tmp/.X11-unix:/tmp/.X11-unix', '-v', f'{self.project_absolute_path}/map:/map', '-p', f'{port}:{port}',
            '--rm', 'sumo_docker', 'sumo-gui', '-c', '/map/sumo_map.sumocfg',
            '--remote-port', f'{port}', '--start', '--step-length', f'{step_length}'
        ]
        return subprocess.Popen(cmd)

def main(args=None):
    rclpy.init(args=args)

    sumo_runner = SumoRunner()

    try:
        rclpy.spin(sumo_runner)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        sumo_runner.destroy_node()
        rclpy.shutdown()

