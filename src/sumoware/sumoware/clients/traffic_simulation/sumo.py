from .base import BaseTrafficSimulation
import traci

class SumoTrafficSimulation(BaseTrafficSimulation):
    def __init__(self, host, port):
        self.traci_client = traci.connect(host=host, port=port)

    def simulation_step(self, time = 0):
        self.traci_client.simulationStep(time)
        return True

    def get_time(self):
        return self.traci_client.simulation.getTime()

    def get_vehicle_pos(self, vehicle_id):
        return self.traci_client.vehicle.getPosition(vehicle_id)

    def get_angle(self, vehicle_id):
        return self.traci_client.vehicle.getAngle(vehicle_id)

    def get_id_list(self):
        return self.traci_client.vehicle.getIDList()

    def add_legacy(self, vehicle_id):
        self.traci_client.vehicle.addLegacy(vehicle_id, "")
        self.traci_client.vehicle.setSpeedMode(vehicle_id, 32)
        return True

    def move_to_xy(self, vehicle_id, x, y, angle):
        self.traci_client.vehicle.moveToXY(vehicle_id, "", 0, x, y, angle=angle, keepRoute=2)
        return True

    def set_speed(self, vehicle_id, speed):
        self.traci_client.vehicle.setSpeed(vehicle_id, speed)
        return True
    
    def get_speed(self, vehicle_id):
        return self.traci_client.vehicle.getSpeed(vehicle_id)
    
    def close(self):
        self.traci_client.close()
        return True