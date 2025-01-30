import math
import random
import rclpy
from rclpy.node import Node

from .clients.traffic_simulation.sumo import SumoTrafficSimulation
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion
from tier4_simulation_msgs.msg import DummyObject
from .models.vehicle import Vehicle
from autoware_vehicle_msgs.msg import VelocityReport

from std_msgs.msg import String

class SumowareBridge(Node):

    def __init__(self):
        super().__init__('sumoware_bridge')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 4001)
        self.declare_parameter('timer_period', 0.03)
        self.declare_parameter('time_step', 0.01)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        timer_period = self.get_parameter('timer_period').value
        self.time_step = self.get_parameter('time_step').value

        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self.clock_publisher.publish(Clock(clock=Time(sec=0, nanosec=0)))
        self.perception_publisher = self.create_publisher(DummyObject, '/simulation/dummy_perception_publisher/object_info', 10)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.aw_initial_pose_callback,
            10)
        self.create_subscription(
            TFMessage,
            '/tf',
            self.aw_tf_callback,
            10)
        self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.aw_velocity_callback,
            10
        )
        self.create_subscription(
            String,
            '/sumoware/shutdown',
            self.shutdown_callback,
            10
        )

        self.timer = self.create_timer(timer_period, self.update_loop)

        self.ego_id = None
        self.vehicle_map: dict[str, Vehicle] = {}
        self.latest_aw_tf_msg = None
        self.latest_aw_speed = 0.0
        self.aw_init = False
        self.time = 0.0

        self.traffic_simulation = SumoTrafficSimulation(host, port)
        self.get_logger().info('Sumoware bridge initialized')

    def shutdown_callback(self, _):
        self.get_logger().info('Shutting down...')
        self.traffic_simulation.close()
        raise SystemExit

    def run_time_step(self):
        self.time += self.time_step
        sec = int(self.time)
        nsec = int((self.time - sec) * 1e9)
        self.clock_publisher.publish(Clock(clock=Time(sec=sec, nanosec=nsec)))
        self.traffic_simulation.simulation_step(self.time)

    def aw_initial_pose_callback(self, msg):
        self.get_logger().info('Initial pose received')
    
    def aw_tf_callback(self, msg):
        self.latest_aw_tf_msg = msg

    def aw_velocity_callback(self, msg):
        self.latest_aw_speed = msg.longitudinal_velocity
        if not self.ego_id:
            random_id = f"ego_{random.randint(0, 1000)}"
            self.traffic_simulation.add_legacy(random_id)
            self.ego_id = random_id

    def update_sumo_ego(self):
        if not self.latest_aw_tf_msg:
            return
        position = self.latest_aw_tf_msg.transforms[0].transform.translation
        orientation = self.latest_aw_tf_msg.transforms[0].transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if self.ego_id:
            angle = -yaw*180/math.pi+90
            offset = 3.79
            x = position.x + offset * math.cos(yaw)
            y = position.y + offset * math.sin(yaw)
            self.traffic_simulation.move_to_xy(self.ego_id, x, y, angle)
            self.traffic_simulation.set_speed(self.ego_id, self.latest_aw_speed)

    def pub_sumo_vehicles(self, vehicle_ids):
        time = self.traffic_simulation.get_time() + self.time_step
        sec = int(time)
        nsec = int((time - sec) * 1e9)
        for vehicle in vehicle_ids:
            if vehicle == self.ego_id:
                continue
            action = 1
            if vehicle not in self.vehicle_map:
                action = 0
                self.vehicle_map[vehicle] = Vehicle(vehicle, self.traffic_simulation)
            veh = self.vehicle_map[vehicle]
            msg = veh.get_object_msg(action, sec, nsec)
            self.perception_publisher.publish(msg)

    def remove_vehicles(self, vehicle_ids, sec, nsec):
        removed_veh = [self.vehicle_map[vehicle_id] for vehicle_id in self.vehicle_map.keys() if vehicle_id not in vehicle_ids]
        for veh in removed_veh:
            msg = veh.get_object_msg(2, sec, nsec)
            self.perception_publisher.publish(msg)
            del self.vehicle_map[veh.id]
    
    def update_loop(self):
        if self.ego_id:
            time = self.traffic_simulation.get_time() + self.time_step
            sec = int(time)
            nsec = int((time - sec) * 1e9)

            vehicle_ids = self.traffic_simulation.get_id_list()

            self.pub_sumo_vehicles(vehicle_ids)
            self.update_sumo_ego()

            self.remove_vehicles(vehicle_ids, sec, nsec)
        
        self.run_time_step()

def main(args=None):
    rclpy.init(args=args)

    sumoware_bridge = SumowareBridge()

    try:
        rclpy.spin(sumoware_bridge)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        sumoware_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()