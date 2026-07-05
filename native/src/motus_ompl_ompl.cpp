#include "motus_ompl.h"

#ifdef MOTUS_HAS_OMPL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

extern void motus_set_last_error(const char*);

static bool IsValid(const ob::State* state, int dims, motus_ompl_validity_fn validity, void* userdata)
{
    if (!validity) return true;
    const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
    return validity(rv->values, dims, userdata) != 0;
}

static bool MotionValid(
    const ob::State* s1, const ob::State* s2, int dims, double step_size,
    motus_ompl_validity_fn validity, motus_ompl_motion_validity_fn motion_validity, void* userdata)
{
    const auto* rv1 = s1->as<ob::RealVectorStateSpace::StateType>();
    const auto* rv2 = s2->as<ob::RealVectorStateSpace::StateType>();
    if (motion_validity)
        return motion_validity(rv1->values, rv2->values, dims, userdata) != 0;

    double maxDelta = 0.0;
    for (int i = 0; i < dims; ++i)
        maxDelta = std::max(maxDelta, std::abs(rv2->values[i] - rv1->values[i]));
    const int steps = std::max(1, static_cast<int>(std::ceil(maxDelta / std::max(step_size, 1e-9))));
    for (int s = 0; s <= steps; ++s)
    {
        const double alpha = static_cast<double>(s) / steps;
        double q[64];
        if (dims > 64) return false;
        for (int i = 0; i < dims; ++i)
            q[i] = rv1->values[i] + alpha * (rv2->values[i] - rv1->values[i]);
        if (validity && validity(q, dims, userdata) == 0)
            return false;
    }
    return true;
}

static int WritePath(const og::PathGeometric& path, int dims, int max_states, double* out_path, int* out_count)
{
    const auto& states = path.getStates();
    int written = 0;
    for (const auto* st : states)
    {
        if (written >= max_states) break;
        const auto* rv = st->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < dims; ++i)
            out_path[written * dims + i] = rv->values[i];
        ++written;
    }
    *out_count = written;
    return written > 0 ? MOTUS_OMPL_OK : MOTUS_OMPL_ERR;
}

extern "C" {

int motus_ompl_is_available(void) { return 1; }

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
    (void)goal_bias;
    if (!low || !high || !start || !goal || !out_path || !out_count || dims <= 0 || max_states <= 0)
    {
        motus_set_last_error("invalid arguments");
        return MOTUS_OMPL_ERR;
    }

    *out_count = 0;

    auto space = std::make_shared<ob::RealVectorStateSpace>(dims);
    ob::RealVectorBounds bounds(dims);
    for (int i = 0; i < dims; ++i)
    {
        bounds.setLow(i, low[i]);
        bounds.setHigh(i, high[i]);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([&](const ob::State* s) {
        return IsValid(s, dims, validity, validity_userdata);
    });
    si->setMotionValidator([&](const ob::State* a, const ob::State* b) {
        return MotionValid(a, b, dims, step_size, validity, motion_validity, validity_userdata);
    });
    si->setStateValidityCheckingResolution(0.01);
    si->setup();

    ob::ScopedState<> startState(space);
    ob::ScopedState<> goalState(space);
    for (int i = 0; i < dims; ++i)
    {
        startState[i] = start[i];
        goalState[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(startState, goalState);

    std::shared_ptr<ob::Planner> planner;
    if (planner_id == MOTUS_OMPL_RRT_STAR)
        planner = std::make_shared<og::RRTstar>(si);
    else
        planner = std::make_shared<og::RRTConnect>(si);

    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setRange(step_size);

    ob::PlannerStatus solved;
    if (max_plan_time_sec > 0.0)
        solved = planner->solve(ob::timedPlannerTerminationCondition(max_plan_time_sec));
    else
        solved = planner->solve(ob::IterationTerminationCondition(
            static_cast<unsigned int>(std::max(1, max_iterations))));

    if (!solved)
    {
        motus_set_last_error("planner failed");
        return MOTUS_OMPL_ERR;
    }

    auto pathBase = pdef->getSolutionPath();
    if (!pathBase)
    {
        motus_set_last_error("no solution path");
        return MOTUS_OMPL_ERR;
    }

    auto path = std::static_pointer_cast<og::PathGeometric>(pathBase);
    const auto stateCount = path->getStateCount();
    path->interpolate(static_cast<unsigned int>(
        std::min(static_cast<std::size_t>(max_states), stateCount)));

    return WritePath(*path, dims, max_states, out_path, out_count);
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
    if (!path || path_count < 2 || !out_path || !out_count || dims <= 0)
        return MOTUS_OMPL_ERR;

    auto space = std::make_shared<ob::RealVectorStateSpace>(dims);
    ob::RealVectorBounds bounds(dims);
    for (int i = 0; i < dims; ++i)
    {
        bounds.setLow(i, -1e6);
        bounds.setHigh(i, 1e6);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([&](const ob::State* s) {
        return IsValid(s, dims, validity, validity_userdata);
    });
    si->setMotionValidator([&](const ob::State* a, const ob::State* b) {
        return MotionValid(a, b, dims, step_size, validity, motion_validity, validity_userdata);
    });
    si->setup();

    og::PathGeometric geom(si);
    for (int i = 0; i < path_count; ++i)
    {
        ob::ScopedState<> st(space);
        for (int j = 0; j < dims; ++j)
            st[j] = path[i * dims + j];
        geom.append(st.get());
    }

    og::PathSimplifier simplifier(si);
    simplifier.simplifyMax(geom);

    return WritePath(geom, dims, max_states, out_path, out_count);
}

} // extern "C"

#endif
