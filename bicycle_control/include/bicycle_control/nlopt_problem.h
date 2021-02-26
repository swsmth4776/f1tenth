#ifndef NLOPT_PROBLEM_H
#define NLOPT_PROBLEM_H

// necessary for importing pi
#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>
#include <nlopt.h>

#include "bicycle_parameters.h"

constexpr int horizon = 20;
constexpr int dimensionality = 4*(horizon + 1) + 2*horizon;

// State:   [x, y, v, yaw]
// Control: [a, beta]
// beta is analogous to turning angle -- this should be transformed
// using beta_inv_transform before the server sends the response
enum class StateEnum: int8_t
{
    X,
    Y,
    V,
    YAW,
    ACC,
    BETA
};

int get_index(int t, StateEnum o);

typedef struct {
    double alpha[6];
    double v_desired, yaw_desired;
} velocity_control_data;

typedef struct {
    double alpha[6];
    double x_desired[horizon];
    double y_desired[horizon];
    double v_desired[horizon];
    double yaw_desired[horizon];
} trajectory_control_data;

typedef struct {
    double dt, lr, lf;
    double initial_state[4];
} problem_parameters;

double beta_transform(double turn_angle, double lr, double lf);

double beta_inv_transform(double beta, double lr, double lf);

double objective_function_velocity_control(unsigned n, const double *x, double *grad, void *objective_data);

double objective_function_trajectory_control(unsigned n, const double *x, double *grad, void *objective_data);

void constraint_init_state(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void constraint_x_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void constraint_y_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void constraint_vel_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void constraint_yaw_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void constraint_beta_zero(unsigned m, double *result, unsigned n, const double* x, double* grad, void* constraint_data);

void set_lb(double lb[], unsigned n);
void set_ub(double ub[], unsigned n);

#endif // NLOPT_PROBLEM_H
