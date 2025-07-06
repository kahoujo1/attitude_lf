#ifndef __CONTROL_DEF_H__
#define __CONTROL_DEF_H__

// trajectory generator MPC
#define TG_PRED_HORIZON 300
#define TG_CTRL_HORIZON 300
#define TG_TS 0.05
#define TG_REF_ERROR 0.001
#define ALIGN_TRAJ_HORIZON 100

// feedback MPC
#define MPC_PRED_HORIZON 50
#define MPC_CTRL_HORIZON 100
#define MPC_TS 0.03
#define PREWINDOW 0

#define X_OFFSET 5.0
#define LEADER_MSG_TIMEOUT 0.5

#endif
