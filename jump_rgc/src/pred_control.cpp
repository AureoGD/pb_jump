#include "jump_rgc/pred_control.h"

void PredControl::SetInternalVariables()
{
    this->nx = this->A.rows();      // # states
    this->nu = this->B.cols();      // # decision variables
    this->ny = this->Ca.rows();     // # outputs for the optimization problem
    this->nxa = this->Aa.rows();    // # aumented states
    this->nc = this->C_cons.rows(); // # constraints

    // std::cout << this->nc << std::endl;
}

void PredControl::ResizeMatrices()
{
    // Model prediction matrices: y̅ = G*u̅ + Phi*x[k]
    this->x.resize(this->ny, 1);
    this->x.setZero();

    this->G.resize(this->ny * this->N, this->nu * this->M);
    this->G.setZero();

    this->Phi.resize(this->ny * N, this->nxa);
    this->Phi.setZero();

    // Auxiliare matrices
    this->aux_mdl.resize(this->ny, this->nu);
    this->aux_mdl.setZero();

    this->aux_cons.resize(this->nc, this->nu);
    this->aux_cons.setZero();

    // Constraist prediction mode: Lc_block - Phi_cons*x[k] <= G_cons*u̅ <= Uc_block - Phi_cons*x[k]

    this->G_cons.resize(this->nc * this->N, this->nu * this->M);
    this->G_cons.setZero();

    this->Phi_cons.resize(this->nc * N, this->nxa);
    this->Phi_cons.setZero();

    this->Uc_block.resize(this->nc * this->N, 1);
    this->Uc_block.setZero();

    this->Lc_block.resize(this->nc * this->N, 1);
    this->Lc_block.setZero();

    // Weight matrices: The cost function mus be writen as J = (r̅-y̅)'*Q_blcok*(r̅-y̅) + u̅'*R_block*u̅

    this->Ref_block.resize(this->ny * this->N, 1);
    this->Ref_block.setZero();

    this->Q_block.resize(this->ny * this->N, this->ny * this->N);
    this->Q_block.setZero();

    this->R_block.resize(this->nu * this->M, this->nu * this->M);
    this->R_block.setZero();
}

void PredControl::SetWeightMatrices(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
    if (Q.rows() != this->ny || Q.cols() != this->ny)
    {
        std::cout << "Wrong Q dimension" << std::endl;
        return;
    }

    if (R.rows() != this->nu || R.cols() != this->nu)
    {
        std::cout << "Wrong R dimension" << std::endl;
        return;
    }

    for (int i = 0; i < this->N; i++)
    {
        this->Q_block.block(i * this->ny, i * this->ny, this->ny, this->ny) = Q;
        if (i < this->M)
            this->R_block.block(i * this->nu, i * this->nu, this->nu, this->nu) = R;
    }
}

void PredControl::SetConsBounds(const Eigen::MatrixXd &Lb, const Eigen::MatrixXd &Ub)
{
    if (Lb.rows() != this->nc || Lb.cols() != 1)
    {
        std::cout << "Wrong Lower Bound dimension" << std::endl;
        return;
    }

    if (Ub.rows() != this->nc || Ub.cols() != 1)
    {
        std::cout << "Wrong Upper Bound dimension" << std::endl;
        return;
    }

    this->Lc = Lb;
    this->Uc = Ub;

    for (int i = 0; i < this->N; i++)
    {
        this->Lc_block.block(i * this->nc, 0, this->nc, 1) = this->Lc;
        this->Uc_block.block(i * this->nc, 0, this->nc, 1) = this->Uc;
    }
}

void PredControl::SetReference(const Eigen::MatrixXd &ref)
{
    if (ref.rows() != this->ny || ref.cols() != 1)
    {
        std::cout << "Wrong Ref dimension" << std::endl;
        return;
    }

    for (int i = 0; i < this->N; i++)
    {
        this->Ref_block.block(i * this->ny, 0, this->ny, 1) = ref;
    }
}

void PredControl::UpdateStates(const Eigen::VectorXd &x)
{
    this->x = x;
}

void PredControl::UpdateOptimizationProblem(Eigen::MatrixXd &H,
                                            Eigen::MatrixXd &F,
                                            Eigen::MatrixXd &Ain,
                                            Eigen::VectorXd &lowerBound,
                                            Eigen::VectorXd &upperBound)
{
    // Update the dynamic matrices
    this->UpdateDynamicModel();

    // update the prediction model
    this->UpdatePredictionModel();

    // define initial Phi values
    this->DefinePhi();

    // Update the optimization problem

    H = 2 * (this->G.transpose() * this->Q_block * this->G + this->R_block);

    F = 2 * (((this->Phi * this->x) - this->Ref_block).transpose()) * this->Q_block * this->G;
    Ain = this->G_cons;

    lowerBound = this->Lc_block - this->Phi_cons * this->x;
    upperBound = this->Uc_block - this->Phi_cons * this->x;
}

void PredControl::DefinePhi()
{
    this->Phi.block(0, 0, this->ny, this->nxa) = this->Ca * this->Aa;
    this->aux_mdl = this->Ca * this->Ba;
}

void PredControl::UpdatePredictionModel()
{
    // this->Phi.block(0, 0, this->ny, this->nxa) = this->Ca * this->Aa;
    // this->aux_mdl = this->Ca * this->Ba;

    if (this->nc == 0)
    {
        for (int i = 0; i < N; i++)
        {
            int j = 0;

            if (i != 0)
            {
                this->Phi.block(i * this->ny, 0, this->ny, this->nxa) = this->Phi.block((i - 1) * this->ny, 0, this->ny, this->nxa) * this->Aa;
                this->aux_mdl = this->Phi.block((i - 1) * this->ny, 0, this->ny, this->nxa) * this->Ba;
            }

            while ((j < this->M) and (i + j < this->N))
            {
                this->G.block((i + j) * this->ny, j * this->nu, this->ny, this->nu) = this->aux_mdl;
                j++;
            }
        }
    }
    else
    {
        this->Phi_cons.block(0, 0, this->nc, this->nxa) = this->C_cons * this->Aa;
        this->aux_cons = this->C_cons * this->Ba;
        for (int i = 0; i < N; i++)
        {
            int j = 0;

            if (i != 0)
            {
                this->Phi.block(i * this->ny, 0, this->ny, this->nxa) = this->Phi.block((i - 1) * this->ny, 0, this->ny, this->nxa) * this->Aa;
                this->aux_mdl = this->Phi.block((i - 1) * this->ny, 0, this->ny, this->nxa) * this->Ba;

                this->Phi_cons.block(i * this->nc, 0, this->nc, this->nxa) = this->Phi_cons.block((i - 1) * this->nc, 0, this->nc, this->nxa) * this->Aa;
                this->aux_cons = this->Phi_cons.block((i - 1) * this->nc, 0, this->nc, this->nxa) * this->Ba;
            }

            while ((j < this->M) and (i + j < this->N))
            {
                this->G.block((i + j) * this->ny, j * this->nu, this->ny, this->nu) = this->aux_mdl;
                this->G_cons.block((i + j) * this->nc, j * this->nu, this->nc, this->nu) = this->aux_cons;
                j++;
            }
        }
    }
}