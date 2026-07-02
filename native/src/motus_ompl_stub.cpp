#include "motus_ompl.h"

/* ponytail: stub until OMPL C++ is linked via CMake MOTUS_USE_OMPL=ON */

int motus_ompl_is_available(void)
{
    return 0;
}

int motus_ompl_rrt_connect(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double step_size,
    double goal_bias,
    motus_ompl_validity_fn validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    (void)dims; (void)low; (void)high; (void)start; (void)goal;
    (void)max_iterations; (void)step_size; (void)goal_bias;
    (void)validity; (void)validity_userdata; (void)out_path; (void)max_states;
    if (out_count) *out_count = 0;
    return MOTUS_OMPL_UNAVAILABLE;
}
