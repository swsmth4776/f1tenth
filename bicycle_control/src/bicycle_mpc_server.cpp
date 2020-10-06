#include "ros/ros.h"
#include <string>

#include "bicycle_control/BicycleMPC.h"
#include "bicycle_ode.h"
#include "nlopt_problem.h"



nlopt_opt opt;
double lb[dimensionality];
double ub[dimensionality];

// initial guess
// Use the ODE solver to integrate state over the horizon, keeping control variables 0
void initial_guess(double* x)
{
    // state = [pos_x, pos_y, vel, yaw, acc, turn]
    state_type state(6);
    for (int t = 0; t < horizon; t++)
    {
        state[0] = x[get_index(t, StateEnum::X)];
        state[1] = x[get_index(t, StateEnum::Y)];
        state[2] = x[get_index(t, StateEnum::V)];
        state[3] = x[get_index(t, StateEnum::YAW)];
        state[4] = 0;
        state[5] = 0;
        controlled_stepper_type controlled_stepper;
        BicycleKinematics bk;
        integrate_adaptive(controlled_stepper, bk, state, 0.0, dt, dt/1000.0);
        x[get_index(t+1, StateEnum::X)] = state[0];
        x[get_index(t+1, StateEnum::Y)] = state[1];
        x[get_index(t+1, StateEnum::V)] = state[2];
        x[get_index(t+1, StateEnum::YAW)] = state[3];
    }
}

// warm start


bool bicycle_mpc_solver(bicycle_control::BicycleMPC::Request  &req,
                        bicycle_control::BicycleMPC::Response &res)
{
    opt = nlopt_create(NLOPT_LD_SLSQP, dimensionality);
    set_lb(lb, dimensionality);
    set_ub(ub, dimensionality);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    ROS_INFO("Desired velocity, yaw: [%f, %f]", (double)req.desired_vel, (double)req.desired_yaw);

    objective_data od{ {1.0, 1.0, 0, 0}, req.desired_vel, req.desired_yaw };
    problem_parameters p{dt, lr, lf, {req.ego_pos_x, req.ego_pos_y, req.ego_vel, req.ego_yaw}};

    nlopt_set_min_objective(opt, objective_function, &od);
//    nlopt_add_equality_mconstraint(opt, 4, constraint_init_state, &p, NULL);
    nlopt_add_equality_mconstraint(opt, horizon, constraint_x_update, &p, NULL);
    nlopt_add_equality_mconstraint(opt, horizon, constraint_y_update, &p, NULL);
    nlopt_add_equality_mconstraint(opt, horizon, constraint_vel_update, &p, NULL);
    nlopt_add_equality_mconstraint(opt, horizon, constraint_yaw_update, &p, NULL);
    nlopt_add_equality_mconstraint(opt, horizon, constraint_beta_zero, &p, NULL);

    // set relative tolerance for optimization variables
    nlopt_set_xtol_rel(opt, 1e-4);
    nlopt_set_ftol_abs(opt, 1e-4);

    // Use the ODE solver to simulate a trajector to provide a reasonable starting guess
    double x[dimensionality];
    for (int i = 0; i < 4; i++)
    {
        x[i] = p.initial_state[i];
    }
    for (int i = 4; i < dimensionality; i++)
    {
        x[i] = 0;
    }
    initial_guess(x);

    double minf = 0;
    nlopt_result result = nlopt_optimize(opt, x, &minf);
    ROS_INFO("NLOPT return code: %d", (int)result);
    if (result < 0 && result != -4)
    {
        ROS_INFO("nlopt failed!\n");
        ROS_INFO("minf is %f\n", minf);
    }
    else
    {
        for (int t = 0; t < horizon; t++)
        {
            ROS_INFO("v[%d]=%f", t, x[get_index(t, StateEnum::V)]);

        }
        double opt_turning_angle = beta_inv_transform(x[get_index(0, StateEnum::BETA)], lr, lf);
        double opt_a = x[get_index(0, StateEnum::ACC)];

        ROS_INFO("found minimum with next acceleration, turning angle [%f, %f] :%0.10g\n",
                opt_a, opt_turning_angle, minf);
        res.acceleration = opt_a;
        res.turning_angle = opt_turning_angle;
    }


    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycle_mpc_server");
    ros::NodeHandle n;

// NLOPT_LD_SLSQP is the sequential least squares quadratic programming algorithm[1]
// It iteratively solves approximations of the optimization problem, by taking
// quadratic approximations of the objective function and affine approximations
// of the constraints.
//
// [1]: https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#slsqp


    ros::ServiceServer service = n.advertiseService("bicycle_mpc", bicycle_mpc_solver);
    ROS_INFO("Ready to solve the bicycle MPC");
    ros::spin();

    return 0;
}
