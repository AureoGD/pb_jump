#ifndef OPT_PROBLEM6_H
#define OPT_PROBLEM6_H

#include "jump_rgc/opt_problem.h"

class OptProblem6 : public OptProblem
{
public:
    OptProblem6(JumpRobot *Robot);

    ~OptProblem6();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    void DefinePhi() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx, Jr, C_aux;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif