#include <nlopt.h>
#include <cmath>
#include <vector>

constexpr int horizon = 20;
constexpr int dimensionality = 4*(horizon + 1) + 2*horizon;

enum class objvar: int8_t
{
    X,
    Y,
    V,
    YAW,
    ACC,
    TURN
};

int get_index(int t, objvar o);

typedef struct {
    double alpha[4];
    double v_desired, yaw_desired;
} objective_data;

typedef struct {
    double dt, lr, lf;
    double initial_state[4];
} problem_parameters;

double beta_transform(double delta, double lr, double lf);

double beta_inv_transform(double beta, double lr, double lf);

double objective_function(unsigned n, const double *x, double *grad, void *objective_data);

void constraint_init_state(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

void constraint_vel_update(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);


