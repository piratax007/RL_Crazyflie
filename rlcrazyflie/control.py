import numpy as np
import os
from stable_baselines3 import PPO


class _RLControlForCrazyflie:
    def __init__(
            self,
            policy_path=os.path.dirname('../assets'),
            hover_rpm=14468.42,
            state=np.zeros((1, 12))
    ):
        self.policy = self._get_policy(policy_path)
        self.observation_space = state
        self.reference_rpm = hover_rpm
        self.calculated_rpm = np.zeros((1, 4))

    def denormalize_actions(self):
        normalize_actions, _state = self.policy.predict(
            self.observation_space,
            deterministic=True,
        )

        self.calculated_rpm[0, :] = np.array(self.reference_rpm*(1+0.05*normalize_actions[0, :]))

    def applicable_pwm(self):
        self.denormalize_actions()
        return tuple(map(lambda rpm: (rpm-4070.3)/0.2685, self.calculated_rpm))

    @staticmethod
    def _get_policy(policy_path: str) -> PPO.policy:
        if os.path.isfile(policy_path + '/best_model.zip'):
            return PPO.load(policy_path + '/best_model.zip')

        raise Exception("[ERROR]: no model under the specified path", policy_path)
