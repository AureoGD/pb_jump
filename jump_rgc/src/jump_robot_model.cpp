#include "jump_rgc/jump_robot_model.h"

JumpRobot::JumpRobot()
{

    this->J_com1.setZero();
    this->J_com2.setZero();
    this->J_foot.setZero();
    this->M.setZero();
    this->C.setZero();
    this->HT_foot.setZero();
    this->HT_com1.setZero();
    this->HT_com2.setZero();

    HT_foot(3, 3) = 1;
    HT_com1(3, 3) = 1;
    HT_com2(3, 3) = 1;
    HT_foot(1, 2) = 1;
    HT_com1(1, 2) = 1;
    HT_com2(1, 2) = 1;

    M(1, 1) = 0.01323;
    C(1, 0) = 0;

    this->m_base = 30.0;
    this->m_upr = 1.5;
    this->m_lwr = 1.5;

    this->m_total = this->m_lwr + this->m_upr + this->m_base;

    this->qU << 1.57, 2.18;
    this->qL << -1.57, -2.18;
}

JumpRobot::~JumpRobot()
{
}

void JumpRobot::UpdateSysMatrices(Eigen::Matrix<double, 2, 1> *_q,
                                  Eigen::Matrix<double, 2, 1> *_qd,
                                  Eigen::Matrix<double, 2, 1> *_b,
                                  Eigen::Matrix<double, 2, 1> *_db)
{
    this->base_pos = (*_b);
    this->base_vel = (*_db);
    this->q = (*_q);
    this->dq = (*_qd);

    this->RobotKinematics();
    this->RobotDynamics();

    this->com_pos(0, 0) = (this->HT_com1(0, 3) * this->m_upr + this->HT_com2(0, 3) * this->m_lwr) / this->m_total;
    this->com_pos(1, 0) = (this->HT_com1(2, 3) * this->m_upr + this->HT_com2(2, 3) * this->m_lwr) / this->m_total;

    this->com_pos_w = this->base_pos + (this->com_pos * this->m_base / this->m_total);

    this->J_com = (this->m_upr * this->J_com1 + this->m_lwr * this->J_com2) / this->m_total;
    this->com_vel = this->J_com * this->dq;

    // std::cout << this->J_com << std::endl;
    // std::cout << this->com_vel << std::endl;

    this->foot_pos(0, 0) = this->HT_foot(0, 3);
    this->foot_pos(1, 0) = this->HT_foot(2, 3);

    this->foot_vel = this->J_foot * this->dq;
}

void JumpRobot::RobotDynamics()
{
    this->M(0, 0) = 0.1348935 + 0.07182 * cos(this->q(1, 0));
    this->M(0, 1) = 0.01323 + 0.03591 * cos(this->q(1, 0));
    this->M(1, 0) = 0.01323 + 0.03591 * cos(this->q(1, 0));

    this->C(0, 0) = -0.07182 * sin(this->q(1, 0)) * this->dq(1, 0);
    this->C(0, 1) = -0.03591 * sin(this->q(1, 0)) * this->dq(1, 0);
    this->C(1, 0) = 0.03591 * sin(this->q(1, 0)) * this->dq(0, 0);

    this->G(0, 0) = 0.5325 * sin(this->q(0, 0)) + 0.126 * sin(this->q(0, 0) + this->q(1, 0));
    this->G(1, 0) = 0.126 * sin(this->q(0, 0) + this->q(1, 0));

    // std::cout << this->M << std::endl;
    // std::cout << this->C << std::endl;
    // std::cout << this->G << std::endl;
}

void JumpRobot::RobotKinematics()
{

    this->HT_foot(0, 0) = -sin(this->q(0, 0) + this->q(1, 0));
    this->HT_foot(0, 1) = -cos(this->q(0, 0) + this->q(1, 0));
    this->HT_foot(0, 3) = -0.25 * sin(this->q(0, 0) + this->q(1, 0)) - 0.2850 * sin(this->q(0, 0));
    this->HT_foot(2, 0) = -cos(this->q(0, 0) + this->q(1, 0));
    this->HT_foot(2, 1) = sin(this->q(0, 0) + this->q(1, 0));
    this->HT_foot(2, 3) = -0.25 * cos(this->q(0, 0) + this->q(1, 0)) - 0.2850 * cos(this->q(0, 0)) - 0.125;

    this->HT_com1(0, 0) = -sin(this->q(0, 0));
    this->HT_com1(0, 1) = -cos(this->q(0, 0));
    this->HT_com1(0, 3) = -0.127 * sin(this->q(0, 0));
    this->HT_com1(2, 0) = -cos(this->q(0, 0));
    this->HT_com1(2, 1) = sin(this->q(0, 0));
    this->HT_com1(2, 3) = -0.127 * cos(this->q(0, 0)) - 0.125;

    this->HT_com2(0, 0) = -sin(this->q(0, 0) + this->q(1, 0));
    this->HT_com2(0, 1) = -cos(this->q(0, 0) + this->q(1, 0));
    this->HT_com2(0, 3) = -0.105 * sin(this->q(0, 0) + this->q(1, 0)) - 0.2850 * sin(this->q(0, 0));
    this->HT_com2(2, 0) = -cos(this->q(0, 0) + this->q(1, 0));
    this->HT_com2(2, 1) = sin(this->q(0, 0) + this->q(1, 0));
    this->HT_com2(2, 3) = -0.105 * cos(this->q(0, 0) + this->q(1, 0)) - 0.2850 * cos(this->q(0, 0)) - 0.125;

    this->J_com1(0, 0) = -0.127 * cos(this->q(0, 0));
    this->J_com1(1, 0) = 0.127 * sin(this->q(0, 0));

    this->J_com2(0, 0) = -0.105 * cos(this->q(0, 0) + this->q(1, 0)) - 0.2850 * cos(this->q(0, 0));
    this->J_com2(0, 1) = -0.105 * cos(this->q(0, 0) + this->q(1, 0));
    this->J_com2(1, 0) = 0.105 * sin(this->q(0, 0) + this->q(1, 0)) + 0.2850 * sin(this->q(0, 0));
    this->J_com2(1, 1) = 0.105 * sin(this->q(0, 0) + this->q(1, 0));

    this->J_foot(0, 0) = -0.25 * cos(this->q(0, 0) + this->q(1, 0)) - 0.2850 * cos(this->q(0, 0));
    this->J_foot(0, 1) = -0.25 * cos(this->q(0, 0) + this->q(1, 0));
    this->J_foot(1, 0) = 0.25 * sin(this->q(0, 0) + this->q(1, 0)) + 0.2850 * sin(this->q(0, 0));
    this->J_foot(1, 1) = 0.25 * sin(this->q(0, 0) + this->q(1, 0));
}