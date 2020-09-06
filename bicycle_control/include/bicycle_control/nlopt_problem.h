#include <nlopt.h>
#include <math.h>

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
}

int get_index(int t, objvar o)
{
    switch (o)
    {
        case X:
            return t*4 + 0;
        case Y;
            return t*4 + 1;
        case V:
            return t*4 + 2;
        case YAW:
            return t*4 + 3;
        case ACC:
            return 4*(horizon+1) + t*2 + 0;
        case TURN:
            return 4*(horizon+1) + t*2 + 1;
    }
}

typedef struct {
    double alpha[4];
    double v_desired, double yaw_desired;
} objective_data;

typedef struct {
    double dt, lr, lf;
    double initial_state[4];
} problem_parameters;

double beta_transform(double delta, double lr, double lf)
{
    return delta;

}

double beta_inv_transform(double beta, double lr, double lf)
{
    return beta;
}


double objective_function(unsigned n, const double *x, double *grad, void *objective_data);

double constraint(unsigned n, const double *x, double *grad, void *data);



