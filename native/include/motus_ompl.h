#pragma once

#ifdef _WIN32
#  define MOTUS_OMPL_API __declspec(dllexport)
#else
#  define MOTUS_OMPL_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 when OMPL C++ is linked; 0 for stub build. */
MOTUS_OMPL_API int motus_ompl_is_available(void);

/* Planner status codes */
#define MOTUS_OMPL_OK 0
#define MOTUS_OMPL_ERR -1
#define MOTUS_OMPL_UNAVAILABLE -2

/*
 * Plan joint-space path with RRT-Connect.
 * dims: number of joints
 * low/high: joint bounds (radians)
 * start/goal: joint vectors length dims
 * out_path: caller buffer dims * max_states
 * out_count: number of states written
 * validity_userdata: passed to validity callback
 * validity: return 1 if state is valid
 */
typedef int (*motus_ompl_validity_fn)(const double* state, int dims, void* userdata);

MOTUS_OMPL_API int motus_ompl_rrt_connect(
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
    int* out_count);

#ifdef __cplusplus
}
#endif
