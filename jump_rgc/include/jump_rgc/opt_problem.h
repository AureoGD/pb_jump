#ifndef OPT_PROBLEM_H
#define OPT_PROBLEM_H

#include "jump_rgc/pred_control.h"
#include "jump_rgc/jump_robot_model.h"

class OptProblem : public PredControl
{
public:
    virtual void SetConstants(double _ts, int _N, int _M, double _kp, double _kd);

    virtual void UpdateModelConstants() = 0;

    virtual void UpdateReferences();

    virtual void UpdateReferences(Eigen::VectorXd ref);

    Eigen::VectorXd qhl;

    double ts, Kp, Kd;
};

#endif