#include "ros/ros.h"
#include "longitudinal_control/LongitudinalMPC.h"
#include <string>
#include <cmath>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;
typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;

class LineKinematics
{
public:
    void operator() ( const state_type &x, state_type &dxdt, const double /* t */)
    {
        dxdt[0] = x[1];
        dxdt[1] = x[2];
        dxdt[2] = 0;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "longitudinal_mpc_client");
    if (argc != 2)
    {
        ROS_INFO("usage: longitudinal_mpc_client V");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<longitudinal_control::LongitudinalMPC>("longitudinal_mpc");

    longitudinal_control::LongitudinalMPC srv;
    srv.request.ego_pos = 0;
    srv.request.ego_vel = 3.5;
    srv.request.desired_vel = std::stod(argv[1]);

    double dt = 0.1;
    double epsilon = 0.01;
    while (std::abs(srv.request.desired_vel - srv.request.ego_vel) > epsilon)
    {
        if (!client.call(srv))
        {
            ROS_ERROR("Failed to call service longitudinal_mpc");
            return 1;
        }

        ROS_INFO("State:       [%f, %f]", (double)srv.request.ego_pos, (double)srv.request.ego_vel);
        ROS_INFO("Acceleration: %f", (double)srv.response.acceleration);

        // x = [pos, vel, u], where u = acceleration
        state_type x(3);
        x[0] = srv.request.ego_pos;
        x[1] = srv.request.ego_vel;
        x[2] = srv.response.acceleration;
        controlled_stepper_type controlled_stepper;
        LineKinematics lk;
        integrate_adaptive(controlled_stepper, lk, x, 0.0, dt, dt/1000.0);

        srv.request.ego_pos = x[0];
        srv.request.ego_vel = x[1];

        //srv.request.ego_pos = srv.request.ego_pos + dt*srv.request.ego_vel + 0.5*dt*dt*srv.response.acceleration;
        //srv.request.ego_vel = srv.request.ego_vel + dt*srv.response.acceleration;
    }

    return 0;
}
