#include "ros/ros.h"
#include "bicycle_control/BicycleTrajectoryMPC.h"
#include "bicycle_control/MotionPrimitive.h"
#include <cmath>
#include <mutex>
#include <string>

#include "bicycle_parameters.h"
#include "bicycle_ode.h"

std::mutex m;
bool first = true;

using namespace boost::numeric::odeint;

void motionPrimitiveCallback(const bicycle_control::MotionPrimitive& msg)
{
    std::lock_guard<std::mutex> lock(m);
    if (!first)
    {
        return;
    }
    first = false;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<bicycle_control::BicycleTrajectoryMPC>("bicycle_mpc_trajectory");

    int motion_primitive_horizon = msg.x.size();
    int num_mpc_calls = motion_primitive_horizon - 20;

    
    bicycle_control::BicycleTrajectoryMPC srv;
    srv.request.pos_x[0] = msg.x[0];
    srv.request.pos_y[0] = msg.y[0];
    srv.request.vel[0] = msg.v[0];
    srv.request.yaw[0] = msg.yaw[0];

    for (int c = 0; c < num_mpc_calls; c++)
    {
        copy(msg.x.cbegin()   + c + 1, msg.x.cbegin()   + c + 21, srv.request.pos_x.begin() + 1);
        copy(msg.y.cbegin()   + c + 1, msg.y.cbegin()   + c + 21, srv.request.pos_y.begin() + 1);
        copy(msg.v.cbegin()   + c + 1, msg.v.cbegin()   + c + 21, srv.request.vel.begin()   + 1);
        copy(msg.yaw.cbegin() + c + 1, msg.yaw.cbegin() + c + 21, srv.request.yaw.begin()   + 1);

        if (!client.call(srv))
        {
            ROS_ERROR("Failed to call service bicycle_mpc");
            return;
        }
        ROS_INFO("t = %d", c);
        ROS_INFO("State:   [%f, %f, %f, %f]", (double)srv.request.pos_x[0], (double)srv.request.pos_y[0], (double)srv.request.vel[0], (double)srv.request.yaw[0]);
        ROS_INFO("Control: [%f, %f]\n", (double)srv.response.acceleration, (double)srv.response.turning_angle);

        // x = [pos_x, pos_y, vel, yaw, acc, turn]
        state_type x(6);
        x[0] = srv.request.pos_x[0];
        x[1] = srv.request.pos_y[0];
        x[2] = srv.request.vel[0];
        x[3] = srv.request.yaw[0];
        x[4] = srv.response.acceleration;
        x[5] = srv.response.turning_angle;
        controlled_stepper_type controlled_stepper;
        BicycleKinematics bk;
        integrate_adaptive(controlled_stepper, bk, x, 0.0, dt, dt/1000.0);

        srv.request.pos_x[0] = x[0];
        srv.request.pos_y[0] = x[1];
        srv.request.vel[0] = x[2];
        srv.request.yaw[0] = x[3];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycle_mpc_trajectory_client");
    if (argc != 1)
    {
        ROS_INFO("usage: bicycle_mpc_trajectory_client");
        return 1;
    }

    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("motion_primitive", 1, motionPrimitiveCallback);

    ros::spin();

}
