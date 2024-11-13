import jump_base_model
import os
import numpy as np
import random
from math import cos


class JumpModel(jump_base_model.BaseJumpModel):
    def __init__(self, N=10, M=1):
        super().__init__(_N=N, _M=M)
        # Paths and Constants
        self.model_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "model/jump_2.urdf"
        )
        self.AC_JOINT_LIST = (2, 3)
        self.JOINT_ST_LIST = (0, 1, 2, 3)
        self.JOINT_MODEL_NUM = len(self.JOINT_ST_LIST)

        self.q0 = np.array([0, 0.65, -np.pi * 30 / 180, np.pi * 45 / 180])
        self.joint_p_max = [0, 0.7, 0 * np.pi / 180, 120 * np.pi / 180]
        self.joint_p_min = [0, 0.5, -55 * np.pi / 180, 60 * np.pi / 180]

        self.last_b_x = 0
        self.delta_x_weight = 1.1

    # def action(self, action):
    #     self.actions.appendleft(action)  # Add new action to the front of the deque
    #     self.solved = self.RGC.ChooseRGCPO(action)
    #     if self.solved:
    #         self.qr += self.RGC.delta_qr.reshape(2, 1)

    def update_states(self, aux_q, aux_dq, aux_F_cont):
        self.b[0] = aux_q[0]
        self.b[1] = aux_q[1]

        self.db[0] = aux_dq[0]
        self.db[1] = aux_dq[1]

        self.q[0] = aux_q[2]
        self.q[1] = aux_q[3]

        self.dq[0] = aux_dq[2]
        self.dq[1] = aux_dq[3]

        self.foot_contact_state = aux_F_cont

        self.RGC.UpdateSt(self.q, self.dq, self.qr, self.b, self.db)

    def _reward(self):
        reward = 0

        # Joint error reward
        reward += self.compute_joint_error_reward()

        # Air time reward
        reward += self.compute_air_time_reward()

        # Check if the choosed PO is aproprieted
        reward += self.compute_contact_penalty()

        # Check if the PO was solved
        reward += self.check_constrains_violation()

        # Knee ground collision check
        reward += self.check_knee_ground_collision()

        # Base position in x direction
        reward += self.compute_delta_x()

        # Verify the control mode changes
        reward += self.transition_reward()

        # Check if the foot is inside the ground
        reward = self.check_foot_high(reward)

        self.episode_reward += reward

        return reward

    def compute_delta_x(self):
        delta_x = abs(self.b[0] - self.last_b_x)
        self.last_b_x = self.b[0]
        return self.delta_x_weight * delta_x

    def randon_joint_pos(self):
        # Create a (self.JOINT_MODEL_NUM,) array for joint positions
        q = np.zeros(self.JOINT_MODEL_NUM, dtype=np.float64)

        # Ensure that the feet are not inside the ground
        while (q[1] - 0.25 * cos(q[2] + q[3]) - 0.2850 * cos(q[2]) - 0.125) < 0.1:
            # Generate random joint positions
            for index in range(self.JOINT_MODEL_NUM):
                q[index] = random.uniform(
                    self.joint_p_min[index], self.joint_p_max[index]
                )

        return q
