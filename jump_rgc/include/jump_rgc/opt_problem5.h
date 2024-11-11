#ifndef OPT_PROBLEM5_H
#define OPT_PROBLEM5_H

#include "jump_rgc/opt_problem.h"

class OptProblem5 : public OptProblem
{
public:
    OptProblem5(JumpRobot *Robot);

    ~OptProblem5();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    void DefinePhi() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx, Jr, C_aux;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif