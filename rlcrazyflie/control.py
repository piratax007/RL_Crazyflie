import numpy as np
import os
from stable_baselines3 import PPO


class RLControlForCrazyflie:
    def __init__(
            self,
            hover_rpm,
            policy_path=os.path.dirname('/home/fausto/ros2_ws/src/crazyflierlcontrol/assets/best_model.zip'),
            initial_state=np.zeros((1, 12))
    ):
        self.policy = self._get_policy(policy_path)
        self.observation_space = initial_state
        self.reference_rpm = hover_rpm
        self.calculated_rpm = np.zeros((1, 4))

    def _denormalize_actions(self):
        normalize_actions, _state = self.policy.predict(
            self.observation_space,
            deterministic=True,
        )

        self.calculated_rpm[0, :] = np.array(self.reference_rpm*(1+0.05*normalize_actions[0, :]))

    def applicable_pwm(self):
        self._denormalize_actions()
        return tuple(map(lambda rpm: (rpm-4070.3)/0.2685, self.calculated_rpm[0, :]))

    @staticmethod
    def _get_policy(policy_path: str):
        if os.path.isfile(policy_path + '/best_model.zip'):
            return PPO.load(policy_path + '/best_model.zip')

        raise Exception("[ERROR]: no model under the specified path", policy_path)
