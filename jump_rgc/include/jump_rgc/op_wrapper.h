#ifndef OP_WRAPPER_H
#define OP_WRAPPER_H

#include "jump_rgc/pred_control.h"
#include "jump_rgc/opt_problem.h"
#include "jump_rgc/opt_problem1.h"
#include "jump_rgc/opt_problem2.h"
#include "jump_rgc/opt_problem3.h"
#include "jump_rgc/opt_problem4.h"
#include "jump_rgc/opt_problem5.h"
#include "jump_rgc/opt_problem6.h"

#include "jump_rgc/jump_robot_model.h"

// #include "yaml-cpp/yaml.h"

#include <OsqpEigen/OsqpEigen.h>
#include "math.h"

class Op_Wrapper
{
public:
    Op_Wrapper();

    ~Op_Wrapper();

    void RGCConfig(double _ts, double _Kp, double _Kd);

    int ChooseRGCPO(int npo);

    int SolvePO();

    void UpdateSt(Eigen::Matrix<double, 2, 1> *_q,
                  Eigen::Matrix<double, 2, 1> *_qd,
                  Eigen::Matrix<double, 2, 1> *_qr,
                  Eigen::Matrix<double, 2, 1> *_b,
                  Eigen::Matrix<double, 2, 1> *_db);

    void ResetPO();

    void ClearData();

    Eigen::VectorXd qhl;

    Eigen::Matrix<double, 2, 1> q, qd, qr, delta_qr;
    Eigen::VectorXd QPSolution;

    Eigen::Matrix<double, 2, 1> com_vel;
    Eigen::Matrix<double, 2, 1> com_pos_w;
    Eigen::Matrix<double, 2, 1> foot_pos;
    Eigen::Matrix<double, 2, 1> foot_vel;

private:
    void ConfPO(int index);

    void ClearPO();

    OptProblem1 *optP1;
    OptProblem2 *optP2;
    OptProblem3 *optP3;
    OptProblem4 *optP4;
    OptProblem5 *optP5;
    OptProblem6 *optP6;

    JumpRobot *_JumpRobot;

    OptProblem *op[6];

    int last_op = -1;

    const double g = -9.81;

    bool constraints = 0;

    bool first_conf = 0;

    Eigen::MatrixXd H, F, Ain;

    Eigen::VectorXd x, Ub, Lb;

    Eigen::SparseMatrix<double> hessian_sparse, linearMatrix;

    OsqpEigen::Solver solver;

    bool debug = false;

    bool error_flag = false;
};

#endif