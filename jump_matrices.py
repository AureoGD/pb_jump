import numpy as np
from math import sin, cos


class KinoDynModel:
    def __init__(self):
        self.HT_foot = np.zeros((4, 4), dtype=np.double)
        self.HT_com1 = np.zeros((4, 4), dtype=np.double)
        self.HT_com2 = np.zeros((4, 4), dtype=np.double)

        self.M = np.zeros((2, 2), dtype=np.double)
        self.C = np.zeros((2, 2), dtype=np.double)
        self.J_com = np.zeros((2, 2), dtype=np.double)
        self.J_com1 = np.zeros((2, 2), dtype=np.double)
        self.J_com2 = np.zeros((2, 2), dtype=np.double)
        self.J_foot = np.zeros((2, 2), dtype=np.double)

        self.q = np.zeros((2, 1), dtype=np.double)
        self.dq = np.zeros((2, 1), dtype=np.double)
        self.G = np.zeros((2, 1), dtype=np.double)
        self.com_pos = np.zeros((2, 1), dtype=np.double)
        self.com_pos_w = np.zeros((2, 1), dtype=np.double)
        self.foot_pos = np.zeros((2, 1), dtype=np.double)

        self.M[1, 1] = 0.01323
        self.C[1, 0] = 0

        self.HT_foot[3, 3] = 1
        self.HT_com1[3, 3] = 1
        self.HT_com2[3, 3] = 1

        self.HT_foot[1, 2] = 1
        self.HT_com1[1, 2] = 1
        self.HT_com2[1, 2] = 1

        self.m_base = 30.0
        self.m_upr = 1.5
        self.m_lwr = 1.5

        self.m_total = self.m_lwr + self.m_upr + self.m_base

    def DynamicMtxs(self):
        self.M[0, 0] = 0.1348935 + 0.07182 * cos(self.q[1, 0])
        self.M[0, 1] = 0.01323 + 0.03591 * cos(self.q[1, 0])
        self.M[1, 0] = 0.01323 + 0.03591 * cos(self.q[1, 0])

        self.C[0, 0] = -0.07182 * sin(self.q[1, 0]) * self.dq[1, 0]
        self.C[0, 1] = -0.03591 * sin(self.q[1, 0]) * self.dq[1, 0]
        self.C[1, 0] = 0.03591 * sin(self.q[1, 0]) * self.dq[0, 0]

        self.G[0, 0] = 0.5325 * sin(self.q[0, 0]) + 0.126 * sin(
            self.q[0, 0] + self.q[1, 0]
        )
        self.G[1, 0] = 0.126 * sin(self.q[0, 0] + self.q[1, 0])

    def KinematicMtxs(self):
        self.HT_foot[0, 0] = -sin(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[0, 1] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[0, 3] = -0.25 * sin(self.q[0, 0] + self.q[1, 0]) - 0.2850 * sin(
            self.q[0, 0]
        )
        self.HT_foot[2, 0] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[2, 2] = sin(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[2, 3] = (
            -0.25 * cos(self.q[0, 0] + self.q[1, 0])
            - 0.2850 * cos(self.q[0, 0])
            - 0.125
        )

        self.HT_com1[0, 0] = -sin(self.q[0, 0])
        self.HT_com1[0, 1] = -cos(self.q[0, 0])
        self.HT_com1[0, 3] = -0.127 * sin(self.q[0, 0])
        self.HT_com1[2, 0] = -cos(self.q[0, 0])
        self.HT_com1[2, 2] = sin(self.q[0, 0])
        self.HT_com1[2, 3] = -0.127 * cos(self.q[0, 0]) - 0.125

        self.HT_com2[0, 0] = -sin(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[0, 1] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[0, 3] = -0.105 * sin(self.q[0, 0] + self.q[1, 0]) - 0.2850 * sin(
            self.q[0, 0]
        )
        self.HT_com2[2, 0] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[2, 2] = sin(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[2, 3] = (
            -0.105 * cos(self.q[0, 0] + self.q[1, 0])
            - 0.2850 * cos(self.q[0, 0])
            - 0.125
        )

        self.J_com1[0, 0] = -0.127 * cos(self.q[0, 0])
        self.J_com1[1, 0] = 0.127 * sin(self.q[0, 0])

        self.J_com2[0, 0] = -0.105 * cos(self.q[0, 0] + self.q[1, 0]) - 0.2850 * cos(
            self.q[0, 0]
        )
        self.J_com2[0, 1] = -0.105 * cos(self.q[0, 0] + self.q[1, 0])
        self.J_com2[1, 0] = 0.105 * sin(self.q[0, 0] + self.q[1, 0]) + 0.2850 * sin(
            self.q[0, 0]
        )
        self.J_com2[1, 1] = 0.105 * sin(self.q[0, 0] + self.q[1, 0])

        self.J_foot[0, 0] = -0.25 * cos(self.q[0, 0] + self.q[1, 0]) - 0.2850 * cos(
            self.q[0, 0]
        )
        self.J_foot[0, 1] = -0.25 * cos(self.q[0, 0] + self.q[1, 0])
        self.J_foot[1, 0] = 0.25 * sin(self.q[0, 0] + self.q[1, 0]) + 0.2850 * sin(
            self.q[0, 0]
        )
        self.J_foot[1, 1] = 0.25 * sin(self.q[0, 0] + self.q[1, 0])

    def updateModelMtx(self, b, q, dq):
        self.q = q
        self.dq = dq

        self.KinematicMtxs()
        self.DynamicMtxs()

        # evaluate the CoM position (w.r.t)

        self.com_pos[0, 0] = (
            self.HT_com1[0, 3] * self.m_upr + self.HT_com2[0, 3] * self.m_lwr
        ) / self.m_total
        self.com_pos[1, 0] = (
            self.HT_com1[2, 3] * self.m_upr + self.HT_com2[2, 3] * self.m_lwr
        ) / self.m_total

        self.com_pos_w = (b * self.m_base / self.m_total) + self.com_pos

        self.J_com = (
            self.m_upr * self.J_com1 + self.m_lwr * self.J_com2
        ) / self.m_total

        self.com_vel = np.matmul(self.J_com, self.dq)

        self.foot_pos[0, 0] = self.HT_foot[0, 3]
        self.foot_pos[1, 0] = self.HT_foot[2, 3]

        self.foot_vel = np.matmul(self.J_foot, self.dq)
