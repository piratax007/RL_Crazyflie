from crazyflie_py import Crazyswarm
import numpy as np
from control import RLControlForCrazyflie


class _CrazyflieAPI:
    def __init__(self, drone_id=0):
        self.drone = self._get_drone(drone_id)
        self.GRAVITY = 9.8
        self.CRAZYFLIE_MASS = 0.027
        self.CRAZYFLIE_KF = 3.16e-10
        self.HOVER_RPM = np.sqrt((self.GRAVITY*self.CRAZYFLIE_MASS)/(4*self.CRAZYFLIE_KF))
        self.INITIAL_STATE = np.zeros((1, 12))
        self.control = RLControlForCrazyflie(hover_rpm=self.HOVER_RPM, initial_state=self.INITIAL_STATE)

    @staticmethod
    def _get_drone(crazyflie_id: int):
        swarm = Crazyswarm()
        return swarm.dron.drones[crazyflie_id]


crazyflie = _CrazyflieAPI()
