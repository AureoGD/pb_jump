#ifndef OPT_PROBLEM1_H
#define OPT_PROBLEM1_H

#include "jump_rgc/opt_problem.h"

class OptProblem1 : public OptProblem
{
public:
    OptProblem1(JumpRobot *Robot);

    ~OptProblem1();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif