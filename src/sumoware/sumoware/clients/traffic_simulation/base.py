from abc import abstractmethod


class BaseTrafficSimulation:
    @abstractmethod
    def simulation_step(self, time = 0):
        pass

    @abstractmethod
    def get_time(self):
        pass

    @abstractmethod
    def get_vehicle_pos(self, vehicle_id):
        pass

    @abstractmethod
    def get_angle(self, vehicle_id):
        pass

    @abstractmethod
    def get_id_list(self):
        pass

    @abstractmethod
    def add_legacy(self, vehicle_id):
        pass

    @abstractmethod
    def move_to_xy(self, vehicle_id, x, y, angle):
        pass

    @abstractmethod
    def set_speed(self, vehicle_id, speed):
        pass

    @abstractmethod
    def get_speed(self, vehicle_id):
        pass

    @abstractmethod
    def close(self):
        pass



