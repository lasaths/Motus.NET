#include "motus_ompl.h"

extern void motus_set_last_error(const char*);

int motus_ompl_is_available(void) { return 0; }

int motus_ompl_planner_available(int planner_id)
{
    (void)planner_id;
    return 0;
}

static int motus_ompl_plan_unavailable(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double max_plan_time_sec,
    double step_size,
    double goal_bias,
    int planner_id,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    (void)dims; (void)low; (void)high; (void)start; (void)goal;
    (void)max_iterations; (void)max_plan_time_sec; (void)step_size; (void)goal_bias;
    (void)planner_id; (void)validity; (void)motion_validity; (void)validity_userdata; (void)out_path; (void)max_states;
    if (out_count) *out_count = 0;
    motus_set_last_error("OMPL not linked");
    return MOTUS_OMPL_UNAVAILABLE;
}

int motus_ompl_plan(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double max_plan_time_sec,
    double step_size,
    double goal_bias,
    int planner_id,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    return motus_ompl_plan_unavailable(
        dims, low, high, start, goal, max_iterations, max_plan_time_sec, step_size, goal_bias, planner_id,
        validity, motion_validity, validity_userdata, out_path, max_states, out_count);
}

int motus_ompl_rrt_connect(
    int dims,
    const double* low,
    const double* high,
    const double* start,
    const double* goal,
    int max_iterations,
    double max_plan_time_sec,
    double step_size,
    double goal_bias,
    int planner_id,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    return motus_ompl_plan(
        dims, low, high, start, goal, max_iterations, max_plan_time_sec, step_size, goal_bias, planner_id,
        validity, motion_validity, validity_userdata, out_path, max_states, out_count);
}

int motus_ompl_simplify_path(
    int dims,
    const double* path,
    int path_count,
    double step_size,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count)
{
    (void)dims; (void)path; (void)path_count; (void)step_size;
    (void)validity; (void)motion_validity; (void)validity_userdata; (void)out_path; (void)max_states;
    if (out_count) *out_count = 0;
    return MOTUS_OMPL_UNAVAILABLE;
}
