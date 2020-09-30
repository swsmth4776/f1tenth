#include "ros/ros.h"
#include "bicycle_control/BicycleMPC.h"

#include "nlopt_problem.h"



nlopt_opt opt;
double lb[dimensionality];
double ub[dimensionality];

bool bicycle_mpc_solver(bicycle_control::BicycleMPC::Request  &req,
                        bicycle_control::BicycleMPC::Response &res)
{
    objective_data od{ {1.0, 1.0, 1.0, 1.0}, req.desired_vel, req.desired_yaw };
    problem_parameters p{dt, lr, lf, {req.ego_pos_x, req.ego_pos_y, req.ego_vel, req.ego_yaw}};



    nlopt_set_min_objective(opt, objective_function, &od);
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
    opt = nlopt_create(NLOPT_LD_SLSQP, dimensionality);
    set_lb(lb, dimensionality);
    set_ub(ub, dimensionality);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);


    ros::ServiceServer service = n.advertiseService("bicycle_mpc", bicycle_mpc_solver);
    ROS_INFO("Ready to solve the bicycle MPC");
    ros::spin();

    return 0;
}
