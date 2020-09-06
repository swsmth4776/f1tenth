#include "ros/ros.h"
#include "bicycle_control/BicycleMPC.h"

#include "nlopt_problem.h"



bool bicycle_mpc_solver(bicycle_control::BicycleMPC::Request  &req,
                        bicycle_control::BicycleMPC::Response &res)
{
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycle_mpc_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("bicycle_mpc", bicycle_mpc_solver);
    ROS_INFO("Ready to solve the bicycle MPC");
    ros::spin();

    return 0;
}
