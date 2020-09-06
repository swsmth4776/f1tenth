#include "nlopt_problem.h"

int get_index(int t, objvar o)
{
    switch (o)
    {
        case objvar::X:
            return t*4 + 0;
        case objvar::Y:
            return t*4 + 1;
        case objvar::V:
            return t*4 + 2;
        case objvar::YAW:
            return t*4 + 3;
        case objvar::ACC:
            return 4*(horizon+1) + t*2 + 0;
        case objvar::TURN:
            return 4*(horizon+1) + t*2 + 1;
    }
}

double beta_transform(double delta, double lr, double lf)
{
    return delta;
}

double beta_inv_transform(double beta, double lr, double lf)
{
    return beta;
}

double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    objective_data *d = reinterpret_cast<objective_data*>(data);

    double val = 0;
    for (int t = 0; t < horizon+1; t++)
    {
        val += d->alpha[0]*pow( x[get_index(t, objvar::V)] - d->v_desired, 2);
        val += d->alpha[1]*pow( x[get_index(t, objvar::YAW)] - d->yaw_desired, 2);
    }
    
    for (int t = 0; t < horizon; t++)
    {
        val += d->alpha[2]*pow( x[get_index(t, objvar::ACC)], 2);
        val += d->alpha[3]*pow( x[get_index(t, objvar::TURN)], 2);
    }

    if (!grad.empty())
    {
        for (int t = 0; t < horizon+1; t++)
        {
            grad[get_index(t, objvar::X)] = 0;
            grad[get_index(t, objvar::Y)] = 0;
            grad[get_index(t, objvar::V)] = 2*d->alpha[0]*(x[get_index(t, objvar::V)] - d->v_desired);
            grad[get_index(t, objvar::YAW)] = 2*d->alpha[1]*(x[get_index(t, objvar::YAW)] - d->yaw_desired);
        }
        for (int t = 0; t < horizon; t++)
        {
            grad[get_index(t, objvar::ACC)] = 2*d->alpha[2]*get_index(t, objvar::ACC);
            grad[get_index(t, objvar::TURN)] = 3*d->alpha[2]*get_index(t, objvar::TURN);
        }
    }
    return val;
}

// constraint_init_state
// m = 4
// Constrains x[1] to be equal to the initial state
void constraint_init_state(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    problem_parameters* p = (problem_parameters*) f_data;
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
void constraint_vel_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    problem_parameters* p = (problem_parameters*) f_data;
    for (int t = 0; t < m; t++)
    {
        double v_t1 = x[get_index(t+1, objvar::V)];
        double v_t  = x[get_index(t, objvar::V)];
        double a_t  = x[get_index(t, objvar::ACC)];
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
            grad[t*n + get_index(t+1, objvar::V)] = 1;
            grad[t*n + get_index(t, objvar::V)]   = -1;
            grad[t*n + get_index(t, objvar::ACC)] = -(p->dt);
        }
    }
}






