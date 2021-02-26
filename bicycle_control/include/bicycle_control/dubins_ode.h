#ifndef DUBINS_ODE_H
#define DUBINS_ODE_H

#include <boost/numeric/odeint.hpp>
#include <string>
#include <cmath>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;
typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;

class DubinsKinematics
{
public:
// state: [x, y, v, theta, a, u]
// where
// x' = v cos theta
// y' = v sin theta
// v' = a
// theta' = u
    void operator() ( const state_type &x, state_type &dxdt, const double /* t */)
    {
        dxdt[0] = x[2]*cos(x[3]);
        dxdt[1] = x[2]*sin(x[3]);
        dxdt[2] = x[4];
        dxdt[3] = x[5];

        // Hold controls constant over dt, so these should be 0
        dxdt[4] = 0;
        dxdt[5] = 0;
    }
};

#endif // DUBINS_ODE_H
