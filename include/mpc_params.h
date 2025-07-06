#ifndef __MPC_PARAMS_H__
#define __MPC_PARAMS_H__

struct mpc_params
{
    // MPC cost function parameters
    // Q function parameters (state -> state)
    float cost_Q_pos_vertical = 100;
    float cost_Q_pos_horizontal = 100;
    float cost_Q_velocity_vertical = 5;
    float cost_Q_velocity_horizontal = 5;
    float cost_Q_input_reg = 10;

    // R function parameters (input -> input) the price of input increments
    float cost_R_horizontal = 300;
    float cost_R_vertical = 100;

    // MPC constraints
    float max_input_angle = .5;
    float slack_cost = 10;
    // state constraints
    float max_vel_horizontal = 10;
    float max_angles = 15;
    // input rates
    float max_attitude_ctrl_rate = .1;
    float max_thrust_rate = 0.1;
};



#endif