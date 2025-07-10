#include "controller.h"

#define PRED_TIME 0.2

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

// ---------------------- MPC CONTROLLER ----------------------


MPC::MPC(UAVModel &model)
{
    // initialize the model
    model_ = model;
    ROS_INFO("[LeaderFollowerController/MPC]: initializing MPC\n");
    auto [A, B, C_] = model_.getLinModel(MPC_TS);
    // create incremental model
    MatrixXd Aaug = MatrixXd::Zero(N_STATES + N_INPUTS, N_STATES + N_INPUTS);
    Aaug.block(0, 0, N_STATES, N_STATES) = A;
    Aaug.block(0, N_STATES, N_STATES, N_INPUTS) = B;
    Aaug.block(N_STATES, N_STATES, N_INPUTS, N_INPUTS) = MatrixXd::Identity(N_INPUTS, N_INPUTS);

    MatrixXd Baug = MatrixXd::Zero(N_STATES + N_INPUTS, N_INPUTS);
    Baug.block(0, 0, N_STATES, N_INPUTS) = B;
    Baug.block(N_STATES, 0, N_INPUTS, N_INPUTS) = MatrixXd::Identity(N_INPUTS, N_INPUTS);

    // We are not using the C matrix and coding all cost into Q and R

    ROS_INFO("AAAAAAUG: \n");
    // additive variable of linear (to be exact affine) contraints
    VectorXd b = VectorXd::Zero(N_STATES + N_INPUTS);

    Q = MatrixXd::Zero(N_STATES + N_INPUTS, N_STATES + N_INPUTS);
    Q.block(0, 0, 3, 3) = 1 * Matrix3d::Identity(); // compensate for the position
    Q.block(N_STATES, N_STATES , N_INPUTS, N_INPUTS) = 1 * Matrix3d::Identity(); // input regularization
    ROS_INFO("AAAAAAUG: \n");

    // quadratic input (delta u -> delta u) cost matrix
    R = 1*MatrixXd::Identity(N_INPUTS, N_INPUTS);

    // quadratic cross state/input (x -> u) cost matrix
    MatrixXd S = MatrixXd::Zero(N_INPUTS, N_STATES + N_INPUTS);

    // linear input cost matrix
    Vector3d r = Vector3d::Zero();

    // choose the solver
    ocp_qp_solver_plan_t plan;
    plan.qp_solver =
        // FULL_CONDENSING_HPIPM;
        // FULL_CONDENSING_QPOASES;
        // FULL_CONDENSING_QORE;
        // FULL_CONDENSING_DAQP;
        // FULL_CONDENSING_OOQP;
        PARTIAL_CONDENSING_HPIPM;
    // PARTIAL_CONDENSING_QPDUNES;
    // PARTIAL_CONDENSING_OSQP;
    // PARTIAL_CONDENSING_HPMPC;
    // PARTIAL_CONDENSING_OOQP;

    // init the solver config
    config = ocp_qp_xcond_solver_config_create(plan);
    // set dimensions of the optimization problem
    dims = ocp_qp_dims_create(MPC_PRED_HORIZON);
    ROS_INFO("AAAAAAUG: \n");

    // number of states
    // (the model is incremental -> sum of states and inputs)
    int nx = N_STATES + N_INPUTS;
    // number of bounds on states
    // (we pose constraints on all inputs + 2 states, but the
    // inputs are treated as states in incremental model)
    int nbx = 2 + N_INPUTS;
    // how many softbounds on states out of all bounds on x are posed
    int nsbx = 2;

    // number of inputs
    int nu = N_INPUTS;
    // number of bounds on inputs
    int nbu = N_INPUTS;
    // placeholder for zero inputs
    int nu_e = 0;

    // set dimensions of the optimization problem for each stage
    for (int i = 0; i < MPC_PRED_HORIZON + 1; i++)
    {
    // set number of states
    ocp_qp_dims_set(config, dims, i, "nx", &nx);

    if (i)
    {
        // set number of contraints for states
        ocp_qp_dims_set(config, dims, i, "nbx", &nbx);
        ocp_qp_dims_set(config, dims, i, "nsbx", &nsbx);
    }
    else
    {
        // force the initial value of the state
        ocp_qp_dims_set(config, dims, i, "nbx", &nx);
    }

    if (i < MPC_CTRL_HORIZON)
    {
        // set number of inputs and constraints of input
        ocp_qp_dims_set(config, dims, i, "nu", &nu);
        ocp_qp_dims_set(config, dims, i, "nbu", &nbu);
    }
    else
    {
        // block the input when not in control horizon
        ocp_qp_dims_set(config, dims, i, "nu", &nu_e);
    }
    }

    qpIn = ocp_qp_in_create(dims);

    // apply the same model and cost function for each but last stage
    for (int i = 0; i < MPC_PRED_HORIZON; i++)
    {
    ocp_qp_in_set(config, qpIn, i, (char *)"A", Aaug.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"B", Baug.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"b", b.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"Q", Q.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"S", S.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"R", R.data());
    ocp_qp_in_set(config, qpIn, i, (char *)"r", r.data());
    }

    // apply the cost function for the last stage
    ocp_qp_in_set(config, qpIn, MPC_PRED_HORIZON, (char *)"Q", Q.data());
    ocp_qp_in_set(config, qpIn, MPC_PRED_HORIZON, (char *)"S", S.data());
    ocp_qp_in_set(config, qpIn, MPC_PRED_HORIZON, (char *)"r", r.data());

    solverDims = ocp_qp_xcond_solver_dims_create_from_ocp_qp_dims(config, dims);

    opts = ocp_qp_xcond_solver_opts_create(config, solverDims);

    // set partial condensing option - they are not defined by default
    if (plan.qp_solver == PARTIAL_CONDENSING_HPIPM
        // || plan.qp_solver == PARTIAL_CONDENSING_HPMPC
        // || plan.qp_solver == PARTIAL_CONDENSING_OOQP
        // || plan.qp_solver == PARTIAL_CONDENSING_OSQP
        // || plan.qp_solver ==  PARTIAL_CONDENSING_QPDUNES
    )
    {
    int N2 = 10; // Partial condensing parameter
    ocp_qp_xcond_solver_opts_set(config, (ocp_qp_xcond_solver_opts *)opts, "cond_N", &N2);
    }

    qpOut = ocp_qp_out_create(dims);

    qpSolver = ocp_qp_create(config, solverDims, opts);

}

MPC::~MPC()
{
    // free
    ocp_qp_xcond_solver_dims_free(solverDims);
    ocp_qp_dims_free(dims);
    ocp_qp_xcond_solver_config_free(config);
    ocp_qp_xcond_solver_opts_free((ocp_qp_xcond_solver_opts *)opts);
    ocp_qp_in_free(qpIn);
    ocp_qp_out_free(qpOut);
    ocp_qp_solver_destroy(qpSolver);
}

bool MPC::referenceIsSet()
{
    return refIsSet;
}

void MPC::updateModel(double mass)
{
    UAVModelParameters params = model_.getParams();
    params.m = mass;
    model_.set_params(params);
    auto [A, B, C_] = model_.getLinModel(MPC_TS);
    // create incremental model
    // create incremental model
    MatrixXd Aaug = MatrixXd::Zero(N_STATES + N_INPUTS, N_STATES + N_INPUTS);
    Aaug.block(0, 0, N_STATES, N_STATES) = A;
    Aaug.block(0, N_STATES, N_STATES, N_INPUTS) = B;
    Aaug.block(N_STATES, N_STATES, N_INPUTS, N_INPUTS) = MatrixXd::Identity(N_INPUTS, N_INPUTS);

    // apply the same model and cost function for each but last stage
    for (int i = 0; i < MPC_PRED_HORIZON; i++)
    {
    ocp_qp_in_set(config, qpIn, i, (char *)"A", Aaug.data());
    }
}

std::tuple<VectorXd, bool, std::optional<VectorXd>> MPC::getControlAction()
{
    
    std::optional<VectorXd> statePred = std::nullopt;

    if (!refIsSet)
    {
        VectorXd ctrlAct(3); // ctl action: F_t, tau_x, tau_y
        ctrlAct << 0, 0, 0;
        return {ctrlAct, false, statePred};
    }

    ROS_DEBUG("[LeaderFollowerController/MPC]: Trying to calculate control action\n");

    // index all states that should be hard-set as initial condition
    int idxb0[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; // all states and inputs
    VectorXd stateAug(N_STATES + N_INPUTS);
    stateAug << state, lastCtrlAction; // TODO: shouldnt input be added, because last ctrl actioun is ?
    // set the initial condition
    ocp_qp_in_set(config, qpIn, 0, (char *)"idxbx", idxb0);
    // set upper and lower bounds to the same value to fix the state
    ocp_qp_in_set(config, qpIn, 0, (char *)"lbx", stateAug.data()); 
    ocp_qp_in_set(config, qpIn, 0, (char *)"ubx", stateAug.data());
    // update constraints if required
    if (reqConstrUpdate)
    {
        updateConstraints();
        reqConstrUpdate = false;
    }

    // solve the optimization problem
    int acados_return = ocp_qp_solve(qpSolver, qpIn, qpOut);

    VectorXd ctrlAct(3);
    bool ctrlActValid;
    if (acados_return != ACADOS_SUCCESS)
    {
        ROS_INFO("[LeaderFollowerController/MPC]: qp solver returned status %d. Exiting.\n", acados_return);
        // return neutral control action
        ctrlAct = lastCtrlAction;
        ctrlActValid = false;
    }
    else
    {
    ROS_INFO("[LeaderFollowerController/MPC]: qp solver returned status %d. Success.\n", acados_return);
    int uOffset = 3; // ux is the vector of inputs and states
    ctrlAct << qpOut->ux[1].pa[uOffset + N_STATES], qpOut->ux[1].pa[uOffset + N_STATES + 1], qpOut->ux[1].pa[uOffset + N_STATES + 2];
    ctrlActValid = true;
    
    int predI = (int)(PRED_TIME / MPC_TS);

    Vector3d uavPos;
    //uavPos << qpOut->ux[predI].pa[0 + uOffset], qpOut->ux[predI].pa[1 + uOffset], qpOut->ux[predI].pa[2 + uOffset];

    Vector3d uavVel;
    //uavVel << qpOut->ux[predI].pa[5 + uOffset], qpOut->ux[predI].pa[6 + uOffset], qpOut->ux[predI].pa[7 + uOffset];

    Vector2d uavAcc;
    //uavAcc << qpOut->ux[predI].pa[10 + uOffset], qpOut->ux[predI].pa[11 + uOffset];

    Vector2d loadAtt;
    //loadAtt << qpOut->ux[predI].pa[3 + uOffset], qpOut->ux[predI].pa[4 + uOffset];

    Vector2d loadAttRate;
    //loadAttRate << qpOut->ux[predI].pa[8 + uOffset], qpOut->ux[predI].pa[9 + uOffset];

    VectorXd statePredTmp = VectorXd::Zero(N_STATES);
    //statePredTmp << uavPos, loadAtt, uavVel, loadAttRate, uavAcc;
    statePred = statePredTmp;
    
    }

    lastCtrlAction = ctrlAct;

    return {ctrlAct, ctrlActValid, statePred};
}

void MPC::updateState(VectorXd state_)
{
    state = state_;
    return;
}

void MPC::setParameter(std::string paramName, MatrixXd input)
{
    //ROS_INFO("SETTING PARAMETER %s\n", paramName.c_str());
    if (paramName == "Q")
    {
    if (Q == input)
    {
        return;
    }
    Q = input;
    input = Q;
    for (int i = 0; i <= MPC_PRED_HORIZON; i++)
    {
        ocp_qp_in_set(config, qpIn, i, (char *)paramName.c_str(), Q.data());
    }
    return;
    }
    else if (paramName == "R")
    {
    if (R == input)
    {
        return;
    }
    R = input;
    for (int i = 0; i <= MPC_PRED_HORIZON; i++)
    {
    ocp_qp_in_set(config, qpIn, i, (char *)paramName.c_str(), input.data());
    }
    return;

    }
    else if (paramName == "bu")
    {
    if (ubu != input)
    {
        ubu = input;
        lbu = -input;
        reqConstrUpdate = true;
    }
    return;
    }
    else if (paramName == "bui")
    {
    if (ubui != input)
    {
        ubui = input;
        lbui = -input;
        reqConstrUpdate = true;
    }
    return;
    }
    return;
}


void MPC::setParameter(std::string paramName, double input)
{
    if (paramName == "Z") {
        if (velSlackCost != input) {
            velSlackCost = input;
            reqConstrUpdate = true;
        }
    } else if (paramName == "maxVelHrz") {
        if (maxVelHrz != input) {
            maxVelHrz = input;
            reqConstrUpdate = true;
        }
    }
    return;
}


void MPC::updateConstraints()
{
    // indexes of all states that should be constrained
    // 3 .. x velocity
    // 4 .. y velocity
    int idxbx[] = {3, 4, 10, 11, 12}; 
    // indexes of the elements of `idbx` with soft constrints
    int idxsbx[] = {2, 3}; // soft constraint the velocity
    // indexes of all control actions that should be constrained
    int idxbu[] = {0, 1, 2};

    // bounds on states
    VectorXd ubv = VectorXd::Ones(2);
    // velocity
    ubv(0) = maxVelHrz;
    ubv(1) = maxVelHrz;
    // angles 
    // ubv(1) = max_angles;
    // ubv(3) = max_angles;
    VectorXd lbv = -ubv;

    VectorXd ubx = VectorXd::Zero(2 + N_INPUTS);
    VectorXd lbx = VectorXd::Zero(2 + N_INPUTS);
    // ROS_INFO("UPDATING CONSTRAINTS\n");
    // ROS_INFO("UBV: %.3f, %.3f, %.3f, %.3f \n", ubv(0), ubv(1), ubv(2), ubv(3));
    // ROS_INFO("UBU: %.3f, %.3f, %.3f\n", ubu(0), ubu(1), ubu(2));
    ubx << ubv, ubu;
    lbx << lbv, lbu;


    // quadratic cost function matrix of slack variables of horizontal velocity
    Vector2d Z = velSlackCost * Vector2d::Ones();

    for (int i = 0; i <= MPC_PRED_HORIZON; i++)
    {

    if (i <= MPC_CTRL_HORIZON)
    {
        ocp_qp_in_set(config, qpIn, i, (char *)"idxbu", idxbu);
        ocp_qp_in_set(config, qpIn, i, (char *)"ubu", ubui.data());
        ocp_qp_in_set(config, qpIn, i, (char *)"lbu", lbui.data());
    }

    if (i)
    {
        ocp_qp_in_set(config, qpIn, i, (char *)"idxbx", idxbx);
        ocp_qp_in_set(config, qpIn, i, (char *)"ubx", ubx.data());
        ocp_qp_in_set(config, qpIn, i, (char *)"lbx", lbx.data());

        ocp_qp_in_set(config, qpIn, i, (char *)"idxs", idxsbx);
        ocp_qp_in_set(config, qpIn, i, (char *)"Zl", Z.data());
        ocp_qp_in_set(config, qpIn, i, (char *)"Zu", Z.data());
    }
    }

    return;
}


void MPC::setReferenceConstant(Vector3d ref)
{
    VectorXd q = VectorXd::Zero(N_STATES + N_INPUTS);
    VectorXd ref_aug = VectorXd::Zero(N_STATES + N_INPUTS);
    ref_aug(0)  = ref(0); // x
    ref_aug(1) = ref(1); // y
    ref_aug(2) = ref(2); // z
    q = -Q * ref_aug;
    refIsSet = true;
    VectorXd qAux = VectorXd::Zero(N_STATES+N_INPUTS);
    for (int i = 0; i <= MPC_PRED_HORIZON; i++)
    {
        if (i < PREWINDOW)
        {
            ocp_qp_in_set(config, qpIn, i, (char *)"q", qAux.data());
        }
            else
        {
            ocp_qp_in_set(config, qpIn, i, (char *)"q", q.data());
        }
    }
}


void MPC::setReferenceCombined(std::vector<std::pair<VectorXd, MatrixXd>> predictions, double alpha, double dt, Vector3d offset)
{
    refIsSet = true;
    VectorXd q = VectorXd::Zero(N_STATES + N_INPUTS);
    VectorXd ref_aug = VectorXd::Zero(N_STATES + N_INPUTS);
    VectorXd qAux = VectorXd::Zero(N_STATES+N_INPUTS);
    MatrixXd Qscaled = Q;
    MatrixXd Rscaled = R;
    MatrixXd covariance_pinverse;
    double horizontal_scale_factor = 0;
    double vertical_scale_factor = 0;
    double time_decay = 1;
    // TODO: add additional info to the refence
    for (int i = 0; i <= MPC_PRED_HORIZON; i++)
    {
        ref_aug(0) = predictions[i].first(0) - offset.x(); // x
        ref_aug(1) = predictions[i].first(1) - offset.y(); // y
        ref_aug(2) = predictions[i].first(2) - offset.z(); // z
        // calculate the pseudoinverse of the covariance matrix
        covariance_pinverse = calculate_pseudoinverse(predictions[i].second);
        horizontal_scale_factor = covariance_pinverse(0,0);
        vertical_scale_factor = covariance_pinverse(2,2);

        Qscaled(0,0) = Q(0,0) * covariance_pinverse(0,0);
        Qscaled(1,1) = Q(1,1) * covariance_pinverse(1,1);
        Qscaled(2,2) = Q(2,2) * covariance_pinverse(2,2);

        Rscaled(0,0) = R(0,0) * horizontal_scale_factor;
        Rscaled(1,1) = R(1,1) * horizontal_scale_factor;
        Rscaled(2,2) = R(2,2) * vertical_scale_factor;
        time_decay = exp(-alpha * dt * i);
        Qscaled = Qscaled * time_decay;
        Rscaled = Rscaled * time_decay;
        q = -Qscaled * ref_aug;
        if (i < PREWINDOW)
        {
            ocp_qp_in_set(config, qpIn, i, (char *)"q", qAux.data());
        }
        else
        {
            ocp_qp_in_set(config, qpIn, i, (char *)"Q", Qscaled.data());
            ocp_qp_in_set(config, qpIn, i, (char *)"q", q.data());
        }
    }
}
} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin