#include "ros/ros.h"
#include "bicycle_control/BicycleMPC.h"
#include <string>
#include <cmath>

#include "bicycle_parameters.h"
#include "bicycle_ode.h"

using namespace boost::numeric::odeint;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycle_mpc_client");
    if (argc != 2)
    {
        ROS_INFO("usage: bicycle_mpc_client V");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<bicycle_control::BicycleMPC>("bicycle_mpc");

    bicycle_control::BicycleMPC srv;
    srv.request.ego_pos_x = 0;
    srv.request.ego_pos_y = 0;
    srv.request.ego_vel = 3.5;
    srv.request.ego_yaw = 0;
    srv.request.desired_pos_x = 0;
    srv.request.desired_pos_y = 0;
    srv.request.desired_vel = std::stod(argv[1]);
    srv.request.desired_yaw = 0;

    double epsilon = 0.01;
    int count = 0;
    while (std::abs(srv.request.desired_vel - srv.request.ego_vel) > epsilon && count < 30)
    {
        if (!client.call(srv))
        {
            ROS_ERROR("Failed to call service bicycle_mpc");
            return 1;
        }

        ROS_INFO("t = %d", count);
        ROS_INFO("State:   [%f, %f, %f, %f]", (double)srv.request.ego_pos_x, (double)srv.request.ego_pos_y, (double)srv.request.ego_vel, (double)srv.request.ego_yaw);
        ROS_INFO("Control: [%f, %f]\n", (double)srv.response.acceleration, (double)srv.response.turning_angle);

        // x = [pos_x, pos_y, vel, yaw, acc, turn]
        state_type x(6);
        x[0] = srv.request.ego_pos_x;
        x[1] = srv.request.ego_pos_y;
        x[2] = srv.request.ego_vel;
        x[3] = srv.request.ego_yaw;
        x[4] = srv.response.acceleration;
        x[5] = srv.response.turning_angle;
        controlled_stepper_type controlled_stepper;
        BicycleKinematics bk;
        integrate_adaptive(controlled_stepper, bk, x, 0.0, dt, dt/1000.0);

        srv.request.ego_pos_x = x[0];
        srv.request.ego_pos_y = x[1];
        srv.request.ego_vel = x[2];
        srv.request.ego_yaw = x[3];
        count++;
    }

    return 0;
}
