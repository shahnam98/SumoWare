import math
from tier4_simulation_msgs.msg import DummyObject
from ..clients.traffic_simulation.base import BaseTrafficSimulation
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Transform

class Vehicle:
    id_generator = 0
    def __init__(self, id, traffic_simulation: BaseTrafficSimulation):
        self.id = id
        self.uid = Vehicle.id_generator+1
        Vehicle.id_generator += 1
        self.traffic_simulation = traffic_simulation
        self.latest_pos_x = 0
        self.latest_pos_y = 0
        self.latest_angle = 0
        self.latest_speed = 0

    def get_object_msg(self, action, sec, nanosec):
        offset = 3.79
        angle = self.traffic_simulation.get_angle(self.id) if action != 2 else 0
        pos = self.traffic_simulation.get_vehicle_pos(self.id) if action != 2 else [0,0,0]
        speed = self.traffic_simulation.get_speed(self.id) if action != 2 else 0
        self.latest_pos_x = pos[0] - offset * math.cos((-angle + 90) * 3.14159 / 180)
        self.latest_pos_y = pos[1] - offset * math.sin((-angle + 90) * 3.14159 / 180)
        self.latest_angle = angle
        self.latest_speed = speed
        quaternion = quaternion_from_euler(0, 0, (-angle + 90) * 3.14159 / 180)
 

        msg = DummyObject()
        # sec in int, nanosec in int
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = "map"
        msg.id.uuid = [67, 10, 118, 141, 118, 11, 205, 55, 120, 14, 97, 2, 195, 146, 188, self.uid]
        msg.initial_state.pose_covariance.pose.position.x = self.latest_pos_x
        msg.initial_state.pose_covariance.pose.position.y = self.latest_pos_y
        msg.initial_state.pose_covariance.pose.orientation.x = quaternion[0]
        msg.initial_state.pose_covariance.pose.orientation.y = quaternion[1]
        msg.initial_state.pose_covariance.pose.orientation.z = quaternion[2]
        msg.initial_state.pose_covariance.pose.orientation.w = quaternion[3]
        msg.initial_state.twist_covariance.twist.linear.x = self.latest_speed * 1.0
        msg.classification.label = 3
        msg.classification.probability = 1.0
        msg.shape.dimensions.x = 5.0
        msg.shape.dimensions.y = 2.5
        msg.shape.dimensions.z = 3.5
        msg.action = action
        return msg
    
    def get_tf(self):
        msg = Transform()
        msg.translation.x = self.latest_pos_x
        msg.translation.y = self.latest_pos_y
        quaternion = quaternion_from_euler(0, 0, (-self.latest_angle + 90) * 3.14159 / 180)
        msg.rotation.x = quaternion[0]
        msg.rotation.y = quaternion[1]
        msg.rotation.z = quaternion[2]
        msg.rotation.w = quaternion[3]
        return msg