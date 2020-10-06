#ifndef BICYCLE_ODE_H
#define BICYCLE_ODE_H

#include <boost/numeric/odeint.hpp>
#include <string>
#include <cmath>

#include "bicycle_parameters.h"

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;
typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;

class BicycleKinematics
{
public:
    void operator() ( const state_type &x, state_type &dxdt, const double /* t */)
    {
        double b = atan( (lr/(lr+lf)) * tan(x[5]));
        dxdt[0] = x[2]*cos(x[4]+b);
        dxdt[1] = x[2]*sin(x[4]+b);
        dxdt[2] = x[4];
        dxdt[3] = (x[2]/lr) * sin(b);

        // Hold controls constant over dt, so these should be 0
        dxdt[4] = 0;
        dxdt[5] = 0;
    }
};

#endif // BICYCLE_ODE_H
