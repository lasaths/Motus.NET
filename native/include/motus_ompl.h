#pragma once

#include "motus_native.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTUS_OMPL_OK MOTUS_STATUS_OK
#define MOTUS_OMPL_ERR MOTUS_STATUS_ERR
#define MOTUS_OMPL_UNAVAILABLE MOTUS_STATUS_UNAVAILABLE

/* Returns 1 when OMPL C++ is linked; 0 for stub build. */
MOTUS_NATIVE_API int motus_ompl_is_available(void);

typedef int (*motus_ompl_validity_fn)(const double* state, int dims, void* userdata);
typedef int (*motus_ompl_motion_validity_fn)(const double* from, const double* to, int dims, void* userdata);

/* motus_ompl_planner_id */
#define MOTUS_OMPL_RRT_CONNECT  0
#define MOTUS_OMPL_RRT_STAR     1
#define MOTUS_OMPL_AORRTC       2
#define MOTUS_OMPL_LBKPIECE     3
#define MOTUS_OMPL_AIT_STAR     4
#define MOTUS_OMPL_EIT_STAR     5
#define MOTUS_OMPL_BLIT_STAR    6

/* Returns 1 when planner_id is compiled into this motus_native build. */
MOTUS_NATIVE_API int motus_ompl_planner_available(int planner_id);

/*
 * Plan joint-space path.
 * max_plan_time_sec: if > 0, time budget in seconds; else max_iterations is used as iteration count.
 * motion_validity: optional edge check; if null, states are interpolated with step_size when motion_validity unavailable.
 */
MOTUS_NATIVE_API int motus_ompl_plan(
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
    int* out_count);

/* Backward-compatible alias for motus_ompl_plan. */
MOTUS_NATIVE_API int motus_ompl_rrt_connect(
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
    int* out_count);

MOTUS_NATIVE_API int motus_ompl_simplify_path(
    int dims,
    const double* path,
    int path_count,
    double step_size,
    motus_ompl_validity_fn validity,
    motus_ompl_motion_validity_fn motion_validity,
    void* validity_userdata,
    double* out_path,
    int max_states,
    int* out_count);

#ifdef __cplusplus
}
#endif
