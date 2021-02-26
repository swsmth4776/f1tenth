#include "ros/ros.h"
#include "bicycle_control/BicycleMPC.h"
#include "bicycle_control/MotionPrimitive.h"
#include <string>
#include <cmath>

#include "bicycle_parameters.h"
#include "dubins_ode.h"

using namespace boost::numeric::odeint;


double u_control(double t)
{
    if (t < .7) {
        return 0;
    } else if (t < 2.2) {
        return 1;
    } else {
        return 0;
    }
}

double a_control(double t)
{
    if (t < .7) {
        return 1.5;
    } else if (t < 2) {
        return 1;
    } else if (t < 3) {
        return -1;
    } else {
        return 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dubins_motion_primitive_node");
    if (argc != 2)
    {
        ROS_INFO("usage: dubins_motion_primitive_node H");
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher dubins_pub = n.advertise<bicycle_control::MotionPrimitive>("motion_primitive", 1);
    ros::Rate loop_rate(1);

    bicycle_control::MotionPrimitive msg;
    int H = std::stod(argv[1]);
    if (H < 20)
    {
        ROS_ERROR("Horizon must be larger than 20 timesteps");
        return 1;
    }

    double epsilon = 0.01;
    int count = 0;
    int t;
    while (ros::ok())
    {
        msg.x = std::vector<double>{0};
        msg.y = std::vector<double>{0};
        msg.v = std::vector<double>{0};
        msg.yaw = std::vector<double>{M_PI/2.0f};
        // x = [x, y, v, theta, a, u]
        state_type x(6);
        x[0] = msg.x[0];
        x[1] = msg.y[0];
        x[2] = msg.v[0];
        x[3] = msg.yaw[0];
        t = 0;
        for (int i = 1; i < H; i++)
        {
            x[4] = a_control(dt*t);
            x[5] = u_control(dt*t);
            ROS_INFO("t = %d", t);
            ROS_INFO("State:   [%f, %f, %f, %f]", (double)x[0], (double)x[1], (double)x[2], (double)x[3]);
            ROS_INFO("Control: [%f, %f]\n", (double)x[4], (double)x[5]);
            controlled_stepper_type controlled_stepper;
            DubinsKinematics dk;
            integrate_adaptive(controlled_stepper, dk, x, 0.0, dt, dt/1000.0);
            msg.x.push_back(x[0]);
            msg.y.push_back(x[1]);
            msg.v.push_back(x[2]);
            msg.yaw.push_back(x[3]);

            t++;
        }
        dubins_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }





    return 0;
}
