#include "motus_ompl.h"

#ifdef MOTUS_HAS_OMPL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <algorithm>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

static bool IsValid(const ob::State* state, int dims, motus_ompl_validity_fn validity, void* userdata)
{
    if (!validity) return true;
    const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
    return validity(rv->values, dims, userdata) != 0;
}

extern "C" {

int motus_ompl_is_available(void)
{
    return 1;
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
    if (!low || !high || !start || !goal || !out_path || !out_count || dims <= 0 || max_states <= 0)
        return MOTUS_OMPL_ERR;

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

    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    if (goal_bias > 0)
        planner->setRange(step_size);

    ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(
        static_cast<double>(max_iterations) * 0.001));

    if (!solved)
        return MOTUS_OMPL_ERR;

    auto pathBase = pdef->getSolutionPath();
    if (!pathBase)
        return MOTUS_OMPL_ERR;

    auto path = std::static_pointer_cast<og::PathGeometric>(pathBase);
    const auto stateCount = path->getStateCount();
    path->interpolate(std::min(static_cast<unsigned int>(max_states), stateCount));
    const auto& states = path->getStates();
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

} // extern "C"

#endif
