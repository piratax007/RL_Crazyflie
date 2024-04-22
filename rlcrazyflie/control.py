from crazyflie_py import Crazyswarm
import numpy as np
import os
from stable_baselines3 import PPO


def get_policy(policy_path):
    if os.path.isfile(policy_path + '/best_model.zip'):
        return PPO.load(policy_path + '/best_model.zip')

    raise Exception("[ERROR]: no model under the specified path", policy_path)


def main(policy_path):
    swarm = Crazyswarm()
    cf = swarm.allcfs.crazyflies[0]
    policy = get_policy(policy_path)

    # Call to getObservations (VICON)
    obs = np.zeros((1, 12))

    # Here a need a loop?
    action, _states = policy.predict(obs,
                                     deterministic=True
                                     )

    # convert action into PWM

    cf.setParam("motorPowerSet.enable", 1)
    cf.setParam("motorPowerSet.m1", action[0][0])
    cf.setParam("motorPowerSet.m2", action[0][1])
    cf.setParam("motorPowerSet.m3", action[0][2])
    cf.setParam("motorPowerSet.m4", action[0][3])
    cf.setParam("motorPowerSet.enable", 0)

    # How can I land the drone after the mission ends?


if __name__ == '__main__':
    POLICY_PATH = os.path.dirname(
        '../assets'
    )

    main(POLICY_PATH)
