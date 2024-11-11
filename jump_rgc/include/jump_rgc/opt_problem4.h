#ifndef OPT_PROBLEM4_H
#define OPT_PROBLEM4_H

#include "jump_rgc/opt_problem.h"

class OptProblem4 : public OptProblem
{
public:
    OptProblem4(JumpRobot *Robot);

    ~OptProblem4();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif