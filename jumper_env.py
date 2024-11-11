import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import numpy as np
import jump_model
from icecream import ic
import time


class JumperEnv(gym.Env):
    def __init__(self, render=False, show_training=True):
        super(JumperEnv, self).__init__()

        self.robot_mdl = jump_model.JumpModel()
        self._last_frame_time = 0.0
        self._time_step = 0.001
        self._is_render = show_training or render
        self.interations = self.robot_mdl.RGC_ST / self._time_step

        # Initialize the physics client
        self.physics_client = p.connect(p.GUI if self._is_render else p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(self._time_step)

        # Load the plane
        self._load_plane()

        # Load the robot
        self.model, self.num_joints = self._load_robot()
        q0 = self.robot_mdl.q0
        self._initialize_joint_states(q0)

        # Gym variables
        self.action_space = spaces.Discrete(self.robot_mdl.NUM_ACTIONS)
        self.observation_space = spaces.Box(
            low=self.robot_mdl.OBS_LOW_VALUE,
            high=self.robot_mdl.OBS_HIGH_VALUE,
            shape=(self.robot_mdl.NUM_OBS_STATES,),
            dtype=np.float64,
        )

        self.current_step = 0
        self.ep = 0
        self.q_aux = np.zeros((self.robot_mdl.JOINT_MODEL_NUM, 1), dtype=np.float64)
        self.dq_aux = np.zeros((self.robot_mdl.JOINT_MODEL_NUM, 1), dtype=np.float64)
        self.f_cont = 0
        self._last_frame_time = 0

    def _load_plane(self):
        plane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        p.changeDynamics(plane, -1, lateralFriction=1.0)

    def _load_robot(self):
        model = p.loadURDF(self.robot_mdl.model_path, [0, 0, 0], [0, 0, 0, 1])
        number_joints = p.getNumJoints(model)
        p.changeDynamics(model, number_joints - 1, lateralFriction=1.5)
        return model, number_joints

    def _initialize_joint_states(self, q):
        for idx in range(self.num_joints - 1):
            p.resetJointState(self.model, idx, q[idx])
            p.setJointMotorControl2(self.model, idx, p.VELOCITY_CONTROL, force=0)

        self.robot_mdl.init_qr(q[1:3])
        p.enableJointForceTorqueSensor(self.model, self.num_joints - 1)

    def step(self, action):
        self.robot_mdl.action(action)

        # Simulate the environment for N iterations
        for _ in range(int(self.interations)):
            self.tau = self.robot_mdl.compute_tau()
            for idx in range(len(self.robot_mdl.AC_JOINT_LIST)):
                p.setJointMotorControl2(
                    self.model,
                    self.robot_mdl.AC_JOINT_LIST[idx],
                    p.TORQUE_CONTROL,
                    force=self.tau[idx],
                )
            p.stepSimulation()
            if self._is_render:
                time_spent = time.time() - self._last_frame_time
                self._last_frame_time = time.time()
                time_to_sleep = self._time_step - time_spent
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)

        self._get_model_st()
        obs, reward, terminated = self.robot_mdl.end_of_step(
            self.q_aux, self.dq_aux, self.f_cont
        )

        truncated = terminated
        info = {}

        self.current_step += 1
        return obs, reward, terminated, truncated, info

    def reset(self, seed=None):
        super().reset(seed=seed)
        self.robot_mdl.reset_vars()
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(self._time_step)

        self._load_plane()

        self.model, self.num_joints = self._load_robot()
        q = self.robot_mdl.randon_joint_pos()

        self._initialize_joint_states(q)

        p.stepSimulation()

        info = {"ep": self.ep, "Episod reward: ": self.robot_mdl.episode_reward}
        self.ep += 1
        self.current_step = 0

        self.tau = self.robot_mdl.compute_tau()
        self._get_model_st()
        self.robot_mdl.update_states(self.q_aux, self.dq_aux, self.f_cont)

        obs = self.robot_mdl.observation()

        self._last_frame_time = 0
        return obs, info

    def _get_model_st(self):
        for idx in range(self.robot_mdl.JOINT_MODEL_NUM):
            self.q_aux[idx], self.dq_aux[idx], forces, tau = p.getJointState(
                self.model, self.robot_mdl.JOINT_ST_LIST[idx]
            )
        q, dq, forces, tau = p.getJointState(self.model, self.num_joints - 1)
        if forces[2] < self.robot_mdl.Fthreshold:
            self.f_cont = 1
