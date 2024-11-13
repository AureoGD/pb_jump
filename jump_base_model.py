import numpy as np
import os
from math import cos, sqrt
import random
from collections import deque
import sys
from icecream import ic
import time

current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/jump_rgc/build")

import pybind_opWrapper


class BaseJumpModel:
    def __init__(self, _N=10, _M=1):
        #
        self.N = _N
        self.actions = deque([-1] * self.N, maxlen=self.N)
        self.state_history = deque(maxlen=_M)  # Store last N states

        # Gym variables
        self.OBS_LOW_VALUE = 0
        self.OBS_HIGH_VALUE = 1
        self.NUM_OBS_STATES = 19
        self.NUM_ACTIONS = 6

        # Joit control variables
        self.kp, self.kd = 150, 10
        self.Kp, self.Kd = self.kp * np.eye(2), self.kd * np.eye(2)
        self.MAX_TAU = 250
        self.V_MAX_TAU = np.array([[self.MAX_TAU], [self.MAX_TAU]])

        # Robot states variables

        self.q0 = np.array([0.65, -np.pi * 30 / 180, np.pi * 45 / 180])
        self.q = np.zeros((2, 1), dtype=np.double)
        self.dq = np.zeros((2, 1), dtype=np.float64)
        self.b = np.zeros((2, 1), dtype=np.float64)
        self.db = np.zeros((2, 1), dtype=np.float64)
        self.qr = np.zeros((2, 1), dtype=np.float64)
        self.dqr = np.zeros((2, 1), dtype=np.float64)
        self.tau = np.zeros((2, 1))
        self.foot_contact_state = 0

        # RGC
        self.RGC_ST = 0.01
        self.RGC = pybind_opWrapper.Op_Wrapper()
        self.RGC.RGCConfig(self.RGC_ST, self.kp, self.kd)
        self.rgc_solved = 0

        # Joint limits
        self.joint_p_max = [0.7, 0 * np.pi / 180, 120 * np.pi / 180]
        self.joint_p_min = [0.4, -55 * np.pi / 180, 60 * np.pi / 180]

        # Normalize states variables
        self._setup_normalization()

        # Reward variables
        self.episode_reward = 0
        self.min_reward = -50
        self.weight_prohibited_po = 2
        self.weight_joint_error = 0.0075
        self.force_threshold = -10

        self.const_violation_weight = -0.95
        self.n_jumps = 0
        self.time_jump = 0
        self.delta_jump = 0
        self.first_landing = False
        self.weight_jump = 1.15
        self.weight_jump_height = 1.15
        self.jump_height_threshold = 0.1
        self.transition_weight = 1.5

        self.last_obs = np.array((self.NUM_OBS_STATES,))

        self.first_interation = True

    def _setup_normalization(self):
        qd_max, q1_max, q2_max, lin_vel_max = 20, 1.57, 2.18, 5
        self.alfa = np.diagflat(
            [
                0.5 / 20,
                1,
                0.5 / lin_vel_max,
                0.5 / lin_vel_max,
                1,
                1,
                0.5 / lin_vel_max,
                0.5 / lin_vel_max,
                0.5 / q1_max,
                0.5 / q2_max,
                0.5 / qd_max,
                0.5 / qd_max,
                0.5 / q1_max,
                0.5 / q2_max,
                0.5 / self.MAX_TAU,
                0.5 / self.MAX_TAU,
                1 / self.N,
                1 / 10,
                1,
            ]
        )
        self.beta = np.array(
            [
                [0.5],
                [0],
                [0.5],
                [0.5],
                [0],
                [0],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0],
                [0.1],
                [0],
            ]
        )

    def action(self, action):
        self.actions.appendleft(action)  # Add new action to the front of the deque
        self.rgc_solved = self.RGC.ChooseRGCPO(action)
        if self.rgc_solved == 1:
            self.qr += self.RGC.delta_qr.reshape(2, 1)

    def compute_tau(self):
        """Compute the torque (tau) based on joint positions and velocities."""
        joint_errors = self.qr - self.q  # Error between desired and current positions
        joint_velocities = self.dq  # Current joint velocities

        # Example PD controller to compute tau
        self.tau = self.kp * joint_errors + self.kd * joint_velocities

        # Ensure tau respects maximum torque limits
        self.tau = np.clip(self.tau, -self.V_MAX_TAU, self.V_MAX_TAU)
        return self.tau

    def _normalize_states(self, states):
        return np.clip(
            np.matmul(self.alfa, states.reshape(self.NUM_OBS_STATES, 1)) + self.beta,
            self.OBS_LOW_VALUE,
            self.OBS_HIGH_VALUE,
        )

    def _done(self):
        if self.episode_reward <= self.min_reward:
            return True

        # check if the base postion in Z direction is near the ground
        if self.b[1, 0] < 0.2:
            return True
        # otherwise continue
        return False

    def _check_transition(self):
        return sum(
            1
            for i in range(len(self.actions) - 1, 0, -1)
            if self.actions[i] != self.actions[i - 1] and self.actions[i] != -1
        )

    def reset_vars(self):
        self.RGC.ResetPO()
        self.steps = 0
        self.actions = deque([-1] * self.N, maxlen=self.N)
        self.episode_reward = 0
        self.n_jumps = 0
        self.first_interation = True

    def init_qr(self, q):
        self.qr[0] = q[0] - 0.05
        self.qr[1] = q[1] + 0.05

    def compute_joint_error_reward(self):
        joint_errors = self.qr - self.q
        joint_error_abs = np.abs(joint_errors)
        epsilon = 1e-6  # Avoid division issues
        return sum(
            np.clip(self.weight_joint_error / (joint_error_abs + epsilon), 0, 10)
        )

    def compute_air_time_reward(self):
        # Track air time
        self._track_air_time()

        if self.delta_jump > 0.01:
            self.n_jumps += 1
            air_time_reward = self.delta_jump * self.weight_jump + self.n_jumps
            self.delta_jump = 0  # Reset delta_jump after reward
            return air_time_reward
        return 0

    def check_knee_ground_collision(self):
        # Penalty based on the center of mass (b[1, 0])
        if self.b[1, 0] - 0.285 * cos(self.q[0, 0]) - 0.125 < 0.05:
            return -1
        return 0

    def compute_contact_penalty(self):
        # Penalty if foot contact state and certain actions occur
        if self.foot_contact_state and (self.actions[0] in [2, 3]):
            return -self.weight_prohibited_po
        return 0

    def check_foot_high(self, reward):
        """
        Check the foot height and adjust the reward accordingly.
        """
        # Calculate foot height using your existing logic
        foot_height = (
            self.b[1, 0]
            - 0.25 * cos(self.q[0, 0] + self.q[1, 0])
            - 0.2850 * cos(self.q[0, 0])
            - 0.125
        )

        if foot_height > self.jump_height_threshold:
            # Reward is proportional to how close the foot height is to the desired height
            reward += self.weight_jump_height * np.clip(
                foot_height - self.jump_height_threshold,
                a_min=self.jump_height_threshold,
                a_max=self.jump_height_threshold * 3,
            )
        elif foot_height < -0.05:
            # If the foot goes too low, heavily penalize
            reward = -50 - abs(reward)

        return reward

    def check_constrains_violation(self):
        if self.rgc_solved == 0:
            return self.const_violation_weight
        elif self.rgc_solved == -1:
            return self.const_violation_weight * 1.25
        return 0

    def _track_air_time(self):
        """
        Track the air time of the robot based on foot contact (foot_contact_state).
        """
        if self.foot_contact_state and not self.first_landing:
            self.first_landing = True

        # Start jump timer when the foot leaves the ground
        if self.first_landing and not self.foot_contact_state and self.time_jump == 0:
            self.time_jump = time.time()

        # End jump timer and calculate delta_jump when foot touches the ground again
        if self.first_landing and self.foot_contact_state and self.time_jump != 0:
            self.delta_jump = time.time() - self.time_jump
            self.time_jump = 0
            self.first_landing = False  # Reset for next jump

    def transition_reward(self):
        return self.transition_weight * (1 - self.transition_history)

    def end_of_step(self, q_aux, dq_aux, f_cont):
        if np.any(np.isnan(q_aux)) or np.any(np.isnan(dq_aux)):
            self.state_history.append(self.last_obs)
            reward = self.min_reward
            terminated = True
        else:
            self.update_states(q_aux, dq_aux, f_cont)
            obs = self.observation()
            if self.first_interation and self.state_history.maxlen > 1:
                self.fill_deque(obs)
            self.state_history.append(obs)
            self.last_obs = obs
            reward = self._reward()
            terminated = self._done()

        return self.state_history, reward, terminated

    def fill_deque(self, obs):
        for _ in range(self.state_history.maxlen):
            self.state_history.append(obs)
        self.first_interation = False

    def _reward(self):
        pass

    def observation(self):
        self.transition_history = self._check_transition()
        states = np.vstack(
            (
                self.RGC.com_pos_w.reshape(2, 1),
                self.RGC.com_vel.reshape(2, 1),
                self.RGC.foot_pos.reshape(2, 1),
                self.RGC.foot_vel.reshape(2, 1),
                self.q,
                self.dq,
                self.qr,
                self.tau,
                self.transition_history,
                self.actions[0],
                np.array([self.foot_contact_state], dtype=np.float64),
            )
        )

        return self._normalize_states(states).reshape((self.NUM_OBS_STATES,))
