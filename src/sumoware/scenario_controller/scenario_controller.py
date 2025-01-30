import json
import subprocess
import threading
from time import sleep, time
import rclpy
from rclpy.node import Node
from tier4_system_msgs.srv import ChangeOperationMode
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import TwistStamped
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import RouteState

class ScenarioController(Node):
    def __init__(self):
        super().__init__('scenario_controller')

        self.declare_parameter('scenario_file', '')
        self.scenario_file = self.get_parameter('scenario_file').value

        self.declare_parameter('map_file', '')
        self.map_file = self.get_parameter('map_file').value

        self.declare_parameter('output_path', './map')
        self.output_path = self.get_parameter('output_path').value

        self.declare_parameter('project_absolute_path', '')
        self.project_absolute_path = self.get_parameter('project_absolute_path').value

        self.declare_parameter('gpu_support', False)
        self.gpu_support = self.get_parameter('gpu_support').value

        if not self.scenario_file:
            raise ValueError("Scenario file not provided")
        
        if not self.map_file:
            raise ValueError("Map file not provided")
        
        if not self.project_absolute_path:
            raise ValueError("Project absolute path not provided")

        self.initial_speed_publisher = self.create_publisher(TwistStamped, '/simulation/input/initialtwist', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.shutdown_publisher = self.create_publisher(String, '/sumoware/shutdown', 10)

        self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.aw_velocity_callback,
            10
        )
        self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.trajectory_callback,
            10
        )
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            self.route_state_callback,
            10
        )

        self.operation_client = self.create_client(
            ChangeOperationMode, "/system/operation_mode/change_operation_mode"
        )

        with open(self.scenario_file, "r") as f:
            self.scenario_params = json.load(f)

        self.speed_initialized = False
        self.trajectory_valid = False
        self.index = 0
        self.scenario = None

        self.scenario = self.scenario_params[self.index]
        self.run_scenario()
    
    def run_scenario(self):
        self.get_logger().info("Running scenario")
        self.speed_initialized = False
        self.trajectory_valid = False
    
        sumoware = subprocess.Popen(
            [
                "ros2",
                "launch",
                "sumoware",
                "sumoware_launch.py",
                f"map_file:={self.map_file}",
                f"output_path:={self.output_path}",
                f"project_absolute_path:={self.project_absolute_path}",
                f"gpu_support:={self.gpu_support}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        threading.Thread(target=self.read_stdout, args=(sumoware.stdout,), daemon=True).start()
        threading.Thread(target=self.read_stderr, args=(sumoware.stderr,), daemon=True).start()
        
        self.get_logger().info("Waiting autoware to be available...")
        output = subprocess.check_output(["ros2", "service", "list"]).decode("utf-8")
        while "/system/operation_mode/change_autoware_control" not in output:
            output = subprocess.check_output(["ros2", "service", "list"]).decode("utf-8")
            self.get_logger().info("Autoware not available, waiting again...")
            sleep(3)
        self.get_logger().info("Autoware available!")

        self.init_scenario()

    def read_stdout(self, pipe):
        for line in iter(pipe.readline, ""):
            self.get_logger().info(line.strip())
        pipe.close()

    def read_stderr(self, pipe):
        for line in iter(pipe.readline, ""):
            self.get_logger().error(line.strip())
        pipe.close()

    def init_scenario(self):
        initial_quaternion = quaternion_from_euler(0, 0, (-self.scenario["initial_pose"]["theta"] + 90) * 3.14159 / 180)
        goal_quaternion = quaternion_from_euler(0, 0, (-self.scenario["goal_pose"]["theta"] + 90) * 3.14159 / 180)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self.scenario["goal_pose"]["x"]
        goal_pose.pose.position.y = self.scenario["goal_pose"]["y"]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = goal_quaternion[0]
        goal_pose.pose.orientation.y = goal_quaternion[1]
        goal_pose.pose.orientation.z = goal_quaternion[2]
        goal_pose.pose.orientation.w = goal_quaternion[3]
        self.goal_pose_publisher.publish(goal_pose)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = self.scenario["initial_pose"]["x"]
        initial_pose.pose.pose.position.y = self.scenario["initial_pose"]["y"]
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = initial_quaternion[0]
        initial_pose.pose.pose.orientation.y = initial_quaternion[1]
        initial_pose.pose.pose.orientation.z = initial_quaternion[2]
        initial_pose.pose.pose.orientation.w = initial_quaternion[3]
        self.initial_pose_publisher.publish(initial_pose)

    def pub_initial_speed(self):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = self.scenario["initial_speed"]
        self.initial_speed_publisher.publish(message)

    def change_operation_mode(self):
        request = ChangeOperationMode.Request()
        request.mode = 2
        while not self.operation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.operation_client.call_async(request)


    def aw_velocity_callback(self, msg):
        latest_aw_speed = msg.longitudinal_velocity
        if not self.speed_initialized and latest_aw_speed > 0:
            self.speed_initialized = True
            self.pub_initial_speed()

    def trajectory_callback(self, _):
        if not self.trajectory_valid:
            self.get_logger().info('Trajectory received')
            self.change_operation_mode()
            self.trajectory_valid = True

    def route_state_callback(self, msg):
        if msg.state == 3:
            self.get_logger().info("Route state is 3, shutting down...")
            self.shutdown_services()

    def save_dump_file(self):
        with open(f"{self.output_path}/dump.xml", "r") as f:
            dump_file = f.read()

        unix_time = time()
        with open(f"./dumps/dump_{self.scenario['name']}_{unix_time}.xml", "w") as f:
            f.write(dump_file)
    
    def shutdown_services(self):
        self.shutdown_publisher.publish(String(data="shutdown"))
        sleep(5)
        self.index += 1
        self.save_dump_file()
        if self.index < len(self.scenario_params):
            self.scenario = self.scenario_params[self.index]
            self.run_scenario()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ScenarioController()
        rclpy.spin(node)
    finally:
        node.shutdown_services()
        node.destroy_node()
        rclpy.shutdown()
