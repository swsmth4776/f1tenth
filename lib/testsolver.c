// Load and instantiate global namespaces with relevant data and working space.
#include "solver.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

struct MPC_problem {
    double ego_pos; // current ego vehicle position (m)
    double ego_vel; // current ego vehicle velocity (m/s)
    double dt; // sampling rate (s)
    double vel_des; // desired speed (m/s)
    double vel_max; // maximum speed (m/s)
    double vel_min; // minimum speed (m/s)
    double u_cost; // input cost (unitless)
    double u_max; // maximum input (m/s^2)
    double u_min; // minimum input (m/s^2)
};

void set_example_problem(struct MPC_problem * mpc) {
    // set up parameters for an example MPC problem
    mpc->ego_pos = 0;
    mpc->ego_vel = 3.5;
    mpc->dt = 0.1;
    mpc->vel_des = 5;
    mpc->vel_max = 20;
    mpc->vel_min = 0;
    mpc->u_cost = 0.15;
    mpc->u_max = 2;
    mpc->u_min = -2;
}

void load_data(Params * params, struct MPC_problem * mpc) {
    // state cost
    params->Q[0] = 0;
    params->Q[1] = 0;
    params->Q[2] = 0;
    params->Q[3] = 1;
    params->q[0] = 0;
    params->q[1] = -mpc->vel_des;
    // make terminal cost the same
    params->Qf[0] = params->Q[0];
    params->Qf[1] = params->Q[1];
    params->Qf[2] = params->Q[2];
    params->Qf[3] = params->Q[3];
    params->qf[0] = params->q[0];
    params->qf[1] = params->q[1];
    // input cost
    params->R[0] = mpc->u_cost;
    params->r[0] = 0;
    // system dynamics
    params->A[0] = 1;
    params->A[1] = 0;
    params->A[2] = mpc->dt;
    params->A[3] = 1;
    params->B[0] = 0.5*(mpc->dt)*(mpc->dt);
    params->B[1] = mpc->dt;
    // initial state
    params->x1[0] = mpc->ego_pos;
    params->x1[1] = mpc->ego_vel;
    // state constraints
    params->Hx[0] = 0;
    params->Hx[1] = 0;
    params->Hx[2] = 1;
    params->Hx[3] = -1;
    params->hx[0] = mpc->vel_max;
    params->hx[1] = -mpc->vel_min;
    // input constraints
    params->Hu[0] = 1;
    params->Hu[1] = -1;
    params->hu[0] = mpc->u_max;
    params->hu[1] = -mpc->u_min;
}

void print_solution(Vars vars) {
    // In this function, use the optimization result.

    // optimal state / input trajectory
    printf("\nOptimal state / input trajectory: \n");
    int n_u = 20;
    for(int i = 1; i <= n_u; i++) {
        if(i < 10) {
            printf("x_%d =  [%f]  u_%d =  [%f]\n", i, vars.x[i][0], i, vars.u[i][0]);
            printf("       [%f]\n", vars.x[i][1]);
        }
        else {
            printf("x_%d = [%f]  u_%d = [%f]\n", i, vars.x[i][0], i, vars.u[i][0]);
            printf("       [%f]\n", vars.x[i][1]);
        }
    }
    // print final state
    printf("x_21 = [%f]\n", vars.x[21][0]);
    printf("       [%f]\n", vars.x[21][1]);
}

int main(int argc, char **argv) {
    // Set up an example MPC problem
    struct MPC_problem mpc;
    set_example_problem(& mpc);

    // Set basic algorithm parameters.
    set_defaults();
    setup_indexing();
    load_data(& params, & mpc);

    // set solver settings
    settings.verbose = 0;  // disable output of solver progress.
    settings.max_iters = 25;  // can reduce the maximum iteration count, from 25.
    settings.eps = 1e-6;  // can reduce the required objective tolerance, from 1e-6.
    settings.resid_tol = 1e-4;  // can reduce the required residual tolerances, from 1e-4.

    // Solve our problem at high speed!
    int num_iters;
    num_iters = solve();

    // check solution quality
    if(work.converged == 1) {
        printf("\nSolver converged in %d iterations!\n", num_iters);
    }
    else {
        printf("\nSolver did not converge!\n");
    }

    // Recommended: check work.converged == 1.
    print_solution(vars);
}