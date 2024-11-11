#ifndef OPT_PROBLEM3_H
#define OPT_PROBLEM3_H

#include "jump_rgc/opt_problem.h"

class OptProblem3 : public OptProblem
{
public:
    OptProblem3(JumpRobot *Robot);

    ~OptProblem3();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif