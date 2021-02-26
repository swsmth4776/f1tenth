#include "nlopt_problem.h"

int get_index(int t, StateEnum o)
{
    switch (o)
    {
        case StateEnum::X:
            return t*4 + 0;
        case StateEnum::Y:
            return t*4 + 1;
        case StateEnum::V:
            return t*4 + 2;
        case StateEnum::YAW:
            return t*4 + 3;
        case StateEnum::ACC:
            return 4*(horizon+1) + t*2 + 0;
        case StateEnum::BETA:
            return 4*(horizon+1) + t*2 + 1;
    }
}

double beta_transform(double turn_angle, double lr, double lf)
{
    return atan( (lr/(lr+lf)) * tan(turn_angle)); 
}

// beta_transform is invertible in the open interval (-pi/2,pi/2)
// which is reasonable, since it doesn't make sense for wheels 
// to turn more than 90 degrees
double beta_inv_transform(double beta, double lr, double lf)
{
    return atan( ((lr+lf)/lr) * tan(beta)); 
}

double objective_function_velocity_control(unsigned int n, double const*  x, double* grad, void* data)
{
    velocity_control_data *d = reinterpret_cast<velocity_control_data*>(data);

    double val = 0;
    for (int t = 0; t < horizon+1; t++)
    {
        val += d->alpha[0]*pow( x[get_index(t, StateEnum::V)] - d->v_desired, 2);
        val += d->alpha[1]*pow( x[get_index(t, StateEnum::YAW)] - d->yaw_desired, 2);
    }
    
    for (int t = 0; t < horizon; t++)
    {
        val += d->alpha[4]*pow( x[get_index(t, StateEnum::ACC)], 2);
        val += d->alpha[5]*pow( x[get_index(t, StateEnum::BETA)], 2);
    }

    if (grad != nullptr)
    {
        for (int t = 0; t < horizon+1; t++)
        {
            grad[get_index(t, StateEnum::X)] = 0;
            grad[get_index(t, StateEnum::Y)] = 0;
            grad[get_index(t, StateEnum::V)] = 2*d->alpha[0]*(x[get_index(t, StateEnum::V)] - d->v_desired);
            grad[get_index(t, StateEnum::YAW)] = 2*d->alpha[1]*(x[get_index(t, StateEnum::YAW)] - d->yaw_desired);
        }
        for (int t = 0; t < horizon; t++)
        {
            grad[get_index(t, StateEnum::ACC)] = 2*d->alpha[4]*get_index(t, StateEnum::ACC);
            grad[get_index(t, StateEnum::BETA)] = 2*d->alpha[5]*get_index(t, StateEnum::BETA);
        }
    }
    return val;
}


double objective_function_trajectory_control(unsigned int n, double const*  x, double* grad, void* data)
{
    trajectory_control_data* d = reinterpret_cast<trajectory_control_data*>(data);

    double val = 0;
    for (int t = 0; t < horizon+1; t++)
    {
        val += d->alpha[0]*pow( x[get_index(t, StateEnum::X)] - d->x_desired[t], 2);
        val += d->alpha[1]*pow( x[get_index(t, StateEnum::Y)] - d->y_desired[t], 2);
        val += d->alpha[2]*pow( x[get_index(t, StateEnum::V)] - d->v_desired[t], 2);
        val += d->alpha[3]*pow( x[get_index(t, StateEnum::YAW)] - d->yaw_desired[t], 2);
    }
    
    for (int t = 0; t < horizon; t++)
    {
        val += d->alpha[4]*pow( x[get_index(t, StateEnum::ACC)], 2);
        val += d->alpha[5]*pow( x[get_index(t, StateEnum::BETA)], 2);
    }

    if (grad != nullptr)
    {
        for (int t = 0; t < horizon+1; t++)
        {
            grad[get_index(t, StateEnum::X)] = 2*d->alpha[0]*(x[get_index(t, StateEnum::X)] - d->x_desired[t]);
            grad[get_index(t, StateEnum::Y)] = 2*d->alpha[1]*(x[get_index(t, StateEnum::Y)] - d->y_desired[t]);
            grad[get_index(t, StateEnum::V)] = 2*d->alpha[2]*(x[get_index(t, StateEnum::V)] - d->v_desired[t]);
            grad[get_index(t, StateEnum::YAW)] = 2*d->alpha[3]*(x[get_index(t, StateEnum::YAW)] - d->yaw_desired[t]);
        }
        for (int t = 0; t < horizon; t++)
        {
            grad[get_index(t, StateEnum::ACC)] = 2*d->alpha[4]*get_index(t, StateEnum::ACC);
            grad[get_index(t, StateEnum::BETA)] = 2*d->alpha[5]*get_index(t, StateEnum::BETA);
        }
    }
    return val;
}


// constraint_init_state
// m = 4
// Constrains x[1] to be equal to the initial state
void constraint_init_state(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int i = 0; i < m; i++)
    {
        result[i] = x[i] - p->initial_state[i];
    }
    if (grad != NULL)
    {
        for (int i = 0; i < m; i++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[i*n+j] = 0;
            }
            grad[i*n+i] = 1;
        }
    }

}

// m = horizon
// constraint is v[t+1] = v[t] + a[t]dt, t = 0..H
void constraint_vel_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int t = 0; t < m; t++)
    {
        double v_t1 = x[get_index(t+1, StateEnum::V)];
        double v_t  = x[get_index(t, StateEnum::V)];
        double a_t  = x[get_index(t, StateEnum::ACC)];
        result[t] = v_t1 - (v_t + p->dt*a_t);
    }

    if (grad != NULL)
    {
        // For each time
        for (int t = 0; t < m; t++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[t*n+j] = 0;
            }
            // Only three variables are in each constraint
            grad[t*n + get_index(t+1, StateEnum::V)] = 1;
            grad[t*n + get_index(t, StateEnum::V)]   = -1;
            grad[t*n + get_index(t, StateEnum::ACC)] = -(p->dt);
        }
    }
}

// m = horizon
// constraint is x[t+1] = x[t] + dt v[t] cos(yaw[t]+beta[t]), t = 0..H
void constraint_x_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int t = 0; t < m; t++)
    {
        double x_t1 = x[get_index(t+1, StateEnum::X)];
        double x_t = x[get_index(t, StateEnum::X)];
        double v_t  = x[get_index(t, StateEnum::V)];
        double yaw_t  = x[get_index(t, StateEnum::YAW)];
        double beta_t  = x[get_index(t, StateEnum::BETA)];
        result[t] = x_t1 - (x_t + p->dt*v_t*cos(yaw_t+beta_t));
    }

    if (grad != NULL)
    {
        // For each time
        for (int t = 0; t < m; t++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[t*n+j] = 0;
            }
            double v_t  = x[get_index(t, StateEnum::V)];
            double yaw_t  = x[get_index(t, StateEnum::YAW)];
            double beta_t  = x[get_index(t, StateEnum::BETA)];
            // Only five variables are in each constraint
            grad[t*n + get_index(t+1, StateEnum::X)] = 1;
            grad[t*n + get_index(t, StateEnum::X)]   = -1;
            grad[t*n + get_index(t, StateEnum::V)] = -(p->dt*cos(yaw_t+beta_t));
            grad[t*n + get_index(t, StateEnum::YAW)] = p->dt*v_t*sin(yaw_t+beta_t);
            grad[t*n + get_index(t, StateEnum::BETA)] = p->dt*v_t*sin(yaw_t+beta_t);
        }
    }
}

// m = horizon
// constraint is y[t+1] = y[t] + dt v[t] sin(yaw[t]+beta[t]), t = 0..H
void constraint_y_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int t = 0; t < m; t++)
    {
        double y_t1 = x[get_index(t+1, StateEnum::Y)];
        double y_t = x[get_index(t, StateEnum::Y)];
        double v_t  = x[get_index(t, StateEnum::V)];
        double yaw_t  = x[get_index(t, StateEnum::YAW)];
        double beta_t  = x[get_index(t, StateEnum::BETA)];
        result[t] = y_t1 - (y_t + p->dt*v_t*sin(yaw_t+beta_t));
    }

    if (grad != NULL)
    {
        // For each time
        for (int t = 0; t < m; t++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[t*n+j] = 0;
            }
            double v_t  = x[get_index(t, StateEnum::V)];
            double yaw_t  = x[get_index(t, StateEnum::YAW)];
            double beta_t  = x[get_index(t, StateEnum::BETA)];
            // Only five variables are in each constraint
            grad[t*n + get_index(t+1, StateEnum::Y)] = 1;
            grad[t*n + get_index(t, StateEnum::Y)]   = -1;
            grad[t*n + get_index(t, StateEnum::V)] = -(p->dt*sin(yaw_t+beta_t));
            grad[t*n + get_index(t, StateEnum::YAW)] = -(p->dt*v_t*cos(yaw_t+beta_t));
            grad[t*n + get_index(t, StateEnum::BETA)] = -(p->dt*v_t*cos(yaw_t+beta_t));
        }
    }
}

// m = horizon
// constraint is yaw[t+1] = yaw[t] + dt v[t]/lr sin(beta[t]), t = 0..H
void constraint_yaw_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int t = 0; t < m; t++)
    {
        double yaw_t1 = x[get_index(t+1, StateEnum::YAW)];
        double yaw_t = x[get_index(t, StateEnum::YAW)];
        double v_t  = x[get_index(t, StateEnum::V)];
        double beta_t  = x[get_index(t, StateEnum::BETA)];
        result[t] = yaw_t1 - (yaw_t + p->dt*v_t*sin(beta_t)/p->lr);
    }

    if (grad != NULL)
    {
        // For each time
        for (int t = 0; t < m; t++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[t*n+j] = 0;
            }
            double v_t  = x[get_index(t, StateEnum::V)];
            double beta_t  = x[get_index(t, StateEnum::BETA)];
            // Only four variables are in each constraint
            grad[t*n + get_index(t+1, StateEnum::YAW)] = 1;
            grad[t*n + get_index(t, StateEnum::YAW)]   = -1;
            grad[t*n + get_index(t, StateEnum::V)] = -(p->dt*sin(beta_t)/p->lr);
            grad[t*n + get_index(t, StateEnum::BETA)] = -(p->dt*v_t*cos(beta_t));
        }
    }
}


void constraint_beta_zero(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data)
{
    problem_parameters* p = (problem_parameters*) constraint_data;
    for (int t = 0; t < m; t++)
    {
        double yaw_t = x[get_index(t, StateEnum::YAW)];
        result[t] = yaw_t;
    }

    if (grad != NULL)
    {
        // For each time
        for (int t = 0; t < m; t++)
        {
            // Set them all to 0.
            for (int j = 0; j < n; j++)
            {
                grad[t*n+j] = 0;
            }

            grad[t*n + get_index(t, StateEnum::YAW)] = 1;
        }
    }
}

void set_lb(double lb[], unsigned n)
{
    for (int t = 0; t < n; t++)
    {
        lb[t] = -HUGE_VAL;
    }

    for (int t = 0; t < horizon; t++)
    {
        lb[get_index(t, StateEnum::V)] = 0;
        lb[get_index(t, StateEnum::ACC)] = -2;
        lb[get_index(t, StateEnum::BETA)] = beta_transform(-M_PI/12, lr, lf);
    }
}

void set_ub(double ub[], unsigned n)
{
    for (int t = 0; t < n; t++)
    {
        ub[t] = HUGE_VAL;
    }

    for (int t = 0; t < horizon; t++)
    {
        ub[get_index(t, StateEnum::V)] = 20;
        ub[get_index(t, StateEnum::ACC)] = 2;
        ub[get_index(t, StateEnum::BETA)] = beta_transform(M_PI/12, lr, lf);
    }
}

