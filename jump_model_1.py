import jump_base_model
import os
import numpy as np
import random
from math import cos


class JumpModel(jump_base_model.BaseJumpModel):
    def __init__(self, N=10, M=1):
        super().__init__(_N=N, _M=M)
        self.model_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "model/jump.urdf"
        )
        self.AC_JOINT_LIST = (1, 2)
        self.JOINT_ST_LIST = (0, 1, 2)
        self.JOINT_MODEL_NUM = len(self.JOINT_ST_LIST)

    # def action(self, action):
    # self.actions.appendleft(action)  # Add new action to the front of the deque
    # self.rgc_solved = self.RGC.ChooseRGCPO(action)
    # if self.rgc_solved == 1:
    #     self.qr += self.RGC.delta_qr.reshape(2, 1)

    def update_states(self, aux_q, aux_dq, aux_F_cont):
        self.b[1] = aux_q[0]
        self.db[1] = aux_dq[0]

        self.q[0] = aux_q[1]
        self.q[1] = aux_q[2]

        self.dq[0] = aux_dq[1]
        self.dq[1] = aux_dq[2]

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

        # Check if the PO was rgc_solved
        reward += self.check_constrains_violation()

        # Knee ground collision check
        reward += self.check_knee_ground_collision()

        # Check if the foot is inside the ground
        reward = self.check_foot_high(reward)

        self.episode_reward += reward

        return reward

    def randon_joint_pos(self):
        # Create a (self.JOINT_MODEL_NUM,) array for joint positions
        q = np.zeros(self.JOINT_MODEL_NUM, dtype=np.float64)

        # Ensure that the feet are not inside the ground
        while (q[0] - 0.25 * cos(q[1] + q[2]) - 0.2850 * cos(q[1]) - 0.125) < 0.1:
            # Generate random joint positions
            for index in range(self.JOINT_MODEL_NUM):
                q[index] = random.uniform(
                    self.joint_p_min[index], self.joint_p_max[index]
                )

        return q
